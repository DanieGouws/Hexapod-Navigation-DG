import mujoco
import mujoco.viewer
import numpy as np
import time
import os

# Path to your hexapod model XML
MODEL_PATH = "hexapod_Lotriet.xml"

# Load model and create simulation
model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

# Viewer is optional for visual testing
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Simulation started. Press ESC to quit.")

    start_time = time.time()
    while viewer.is_running():
        # Elapsed time
        t = time.time() - start_time

        # Apply sinusoidal test input to each actuator
        for i in range(model.nu):
            data.ctrl[i] = 0.5 * np.sin(2 * np.pi * 0.5 * t + i)  # frequency = 0.5 Hz

        # Step the simulation
        mujoco.mj_step(model, data)

        # Optional: slow it down to real time
        viewer.sync()
