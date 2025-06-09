import mujoco
import mujoco.viewer
import serial
import struct
import threading
import numpy as np
import time

# MODEL_PATH = "hexapod_Lotriet.xml"
MODEL_PATH = "hexapod_CAD_based.xml"
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
micro_to_mujoco_index = [0] * 18
for mujoco_idx, micro_idx in enumerate(mujoco_to_micro_index):
    micro_to_mujoco_index[micro_idx] = mujoco_idx
print(micro_to_mujoco_index)





def serial_listener():
    """Background thread to receive microcontroller commands asynchronously."""
    with serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE, timeout=0.01) as ser:
        line_buffer = b''

        while True:
            first_byte = ser.read(1)

            if not first_byte:
                continue

            # Check for command bytes
            if first_byte[0] == 0xAB:
                print("Found 0xAB")
                angle_data = ser.read(8 * NUM_ACTUATORS)
                if len(angle_data) == 8 * NUM_ACTUATORS:
                    try:
                        print("Trying to unpack angles")
                        mc_angles = struct.unpack('<18d', angle_data)
                        _ = ser.read(8 * NUM_ACTUATORS)  # discard speeds
                        with ctrl_lock:
                            for mujoco_idx in range(NUM_ACTUATORS):
                                mc_idx = mujoco_to_micro_index[mujoco_idx]
                                control_buffer[mujoco_idx] = mc_angles[mc_idx]
                    except struct.error:
                        continue

            # elif first_byte[0] == 0xCD:
            #     id_byte = ser.read(1)
            #     if not id_byte:
            #         continue

            #     micro_id = id_byte[0]
            #     if 0 <= micro_id < NUM_ACTUATORS:
                    
            #         mujoco_id = micro_to_mujoco_index[micro_id]
            #         joint_index = model.actuator_trnid[mujoco_id][0]
            #         joint_qpos = data.qpos[model.jnt_qposadr[joint_index]]
            #         response = struct.pack('<d', joint_qpos)
            #         ser.write(response)
                    # retrieved_actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, mujoco_id)
                    # print(f"Teensy ID {micro_id} → MuJoCo actuator {mujoco_id} ({retrieved_actuator_name}) gives {joint_qpos}")


            elif first_byte[0] == 0xCE:
                with ctrl_lock:
                    joint_angles = []
                    for i in range(NUM_ACTUATORS):
                        mujoco_id = micro_to_mujoco_index[i]
                        joint_index = model.actuator_trnid[mujoco_id][0]
                        angle = data.qpos[model.jnt_qposadr[joint_index]]
                        joint_angles.append(angle)
                response = struct.pack('<18d', *joint_angles)
                ser.write(response)

            else:    #Just print to terminal output
                # Treat as normal text: accumulate until newline
                line_buffer += first_byte
                if first_byte == b'\n':
                    try:
                        print(line_buffer.decode().strip())
                    except UnicodeDecodeError:
                        print("[Decode error]", line_buffer)
                    line_buffer = b''  # reset buffer

# Load and run simulation
model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

# Start background serial thread
serial_thread = threading.Thread(target=serial_listener, daemon=True)
serial_thread.start()

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
