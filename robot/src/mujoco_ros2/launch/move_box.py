#!/usr/bin/env python3
import mujoco
import glfw
import numpy as np
import time

# Loading the scene
xml_path = "model/scene.xml"
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# Window
glfw.init()
window = glfw.create_window(1200, 900, "Conveyor Simulation", None, None)
glfw.make_context_current(window)

# Camera setup
cam = mujoco.MjvCamera()
opt = mujoco.MjvOption()
scn = mujoco.MjvScene(model, maxgeom=10000)
con = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)

cam.lookat[:] = [0.2, 0.1, 0.25]   # focus near belt & box center
cam.distance = 3                # pulled slightly back for full scene
cam.elevation = -25                # look more from above
cam.azimuth = 135                  # angled view to see both arm & conveyor

# IDs
joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "box_slide")

# Simulation parameters
speed = 0.009 # conveyor speed --> box speed
running = True

# Simulation loop
while not glfw.window_should_close(window):
    if running:
        # Move box along Y-axis using the slide joint
        data.qpos[joint_id] += speed

        # Stop box near middle of conveyor
        if data.qpos[joint_id] >= 1.0:
            speed = 0
            running = False

    # Step simulation
    mujoco.mj_step(model, data)

    # Render
    viewport = mujoco.MjrRect(0, 0, 1200, 900)
    mujoco.mjv_updateScene(model, data, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scn)
    mujoco.mjr_render(viewport, scn, con)
    glfw.swap_buffers(window)
    glfw.poll_events()

    time.sleep(0.01)

glfw.terminate()
print("Simulation ended.")
