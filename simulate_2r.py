import mujoco
import mujoco.viewer
import numpy as np
import matplotlib.pyplot as plt
import time

# Load model
model = mujoco.MjModel.from_xml_path('2r_arm.xml')
data = mujoco.MjData(model)

mujoco.mj_resetData(model, data)

# Set initial configuration (arm horizontal to right)
data.qpos[0] = 0.0  # theta1 = 0
data.qpos[1] = 0.0  # theta2 = 0

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.sync()
    input("Press Enter to start simulation...")
    for i in range(10000):
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.001)
    input("Press Enter to close...")

# Storage for data
time_data = []
theta1_data = []
theta2_data = []
theta1_dot_data = []
theta2_dot_data = []
theta1_ddot_data = []
theta2_ddot_data = []

# Simulation parameters
dt = 0.001  # 1ms timestep
sim_time = 3.0  # 3 seconds
n_steps = int(sim_time / dt)

print("Starting simulation...")
print(f"Simulating {sim_time} seconds with dt={dt}")

# Run simulation
for i in range(n_steps):
    # Step the simulation
    mujoco.mj_step(model, data)
    
    # Record data every 10ms (every 10 steps)
    if i % 10 == 0:
        time_data.append(data.time)
        theta1_data.append(data.qpos[0])
        theta2_data.append(data.qpos[1])
        theta1_dot_data.append(data.qvel[0])
        theta2_dot_data.append(data.qvel[1])
        theta1_ddot_data.append(data.qacc[0])
        theta2_ddot_data.append(data.qacc[1])

# Convert to numpy arrays
time_data = np.array(time_data)
theta1_data = np.array(theta1_data)
theta2_data = np.array(theta2_data)
theta1_dot_data = np.array(theta1_dot_data)
theta2_dot_data = np.array(theta2_dot_data)
theta1_ddot_data = np.array(theta1_ddot_data)
theta2_ddot_data = np.array(theta2_ddot_data)

print(f"Simulation complete! Collected {len(time_data)} data points")

# Save data
np.savez('2r_falling_data.npz', 
         time=time_data,
         theta1=theta1_data,
         theta2=theta2_data,
         theta1_dot=theta1_dot_data,
         theta2_dot=theta2_dot_data,
         theta1_ddot=theta1_ddot_data,
         theta2_ddot=theta2_ddot_data)

print("Data saved to '2r_falling_data.npz'")

# Quick plot to visualize
fig, axes = plt.subplots(3, 2, figsize=(12, 10))

axes[0,0].plot(time_data, np.degrees(theta1_data))
axes[0,0].set_ylabel(r'${\theta}_1$ (deg)')
axes[0,0].grid(True)

axes[0,1].plot(time_data, np.degrees(theta2_data))
axes[0,1].set_ylabel(r'${\theta}_2$ (deg)')
axes[0,1].grid(True)

axes[1,0].plot(time_data, theta1_dot_data)
axes[1,0].set_ylabel(r'$\dot{\theta}_1$ (rad/s)')
axes[1,0].grid(True)

axes[1,1].plot(time_data, theta2_dot_data)
axes[1,1].set_ylabel(r'$\dot{\theta}_2$ (rad/s)')
axes[1,1].grid(True)

axes[2,0].plot(time_data, theta1_ddot_data)
axes[2,0].set_ylabel(r'$\ddot{\theta}_1$ (rad/s²)')
axes[2,0].set_xlabel('Time (s)')
axes[2,0].grid(True)

axes[2,1].plot(time_data, theta2_ddot_data)
axes[2,1].set_ylabel(r'$\ddot{\theta}_2$ (rad/s²)')
axes[2,1].set_xlabel('Time (s)')
axes[2,1].grid(True)

plt.tight_layout()
plt.savefig('mujoco_simulation_data.png')
print("Plot saved as 'mujoco_simulation_data.png'")
plt.show()