import mujoco
import mujoco.viewer
import serial
import struct
import threading
import numpy as np
import time

MODEL_PATH = "hexapod_Lotriet.xml"
SERIAL_PORT = "COM5"  # <-- Updated COM port
BAUD_RATE = 115200
NUM_ACTUATORS = 18
SYNC_BYTE = 0xAB

# Shared control buffer
control_buffer = np.zeros(NUM_ACTUATORS, dtype=np.float32)
ctrl_lock = threading.Lock()

# Remapping from microcontroller order to MuJoCo actuator order
# From: [0-17] microcontroller order (counterclockwise legs)
# To: MuJoCo actuator order as defined in the XML
# Format: mujoco_index -> microcontroller_index
mujoco_to_micro_index = [
    12, 13, 14,  # front_left: servo 12-14
    15, 16, 17,  # front_right: servo 15-17
    9, 10, 11,   # mid_left: servo 9-11
    0, 1, 2,     # mid_right: servo 0-2
    6, 7, 8,     # back_left: servo 6-8
    3, 4, 5      # back_right: servo 3-5
]

def serial_listener():
    """Background thread to receive microcontroller commands asynchronously."""
    with serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE, timeout=0.05) as ser:
        while True:
            sync = ser.read(1)
            if not sync or sync[0] != SYNC_BYTE:
                continue

            angle_data = ser.read(4 * NUM_ACTUATORS)
            if len(angle_data) != 4 * NUM_ACTUATORS:
                continue

            try:
                mc_angles = struct.unpack('<18f', angle_data)
            except struct.error:
                continue

            # Optional: read and ignore speeds
            _ = ser.read(4 * NUM_ACTUATORS)

            # Remap angles to match MuJoCo actuator order
            with ctrl_lock:
                for mujoco_idx in range(NUM_ACTUATORS):
                    mc_idx = mujoco_to_micro_index[mujoco_idx]
                    control_buffer[mujoco_idx] = mc_angles[mc_idx]

# Start background serial thread
serial_thread = threading.Thread(target=serial_listener, daemon=True)
serial_thread.start()

# Load and run simulation
model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Simulation running. Press ESC to exit.")

    # Wall-clock time tracker
    t0 = time.time()
    sim_time = 0.0

    while viewer.is_running():
        # Run until simulated time matches real time
        while data.time < (time.time() - t0):
            with ctrl_lock:
                data.ctrl[:] = control_buffer
            mujoco.mj_step(model, data)

        # Sync display (this renders one frame)
        viewer.sync()
