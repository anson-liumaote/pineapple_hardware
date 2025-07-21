import time
import os
import pandas as pd
import matplotlib.pyplot as plt

csv_file = "imu_test.csv"

# Wait for the log file to appear (optional)
wait_time = 0
while not os.path.exists(csv_file) and wait_time < 30:
    print(f"Waiting for {csv_file} to be created...")
    time.sleep(1)
    wait_time += 1

if not os.path.exists(csv_file):
    print(f"File {csv_file} not found. Exiting.")
    exit(1)

# Read the CSV log
df = pd.read_csv(csv_file)

# Plot
plt.figure(figsize=(10, 6))
plt.plot(df['time'], df['ang_vel_x'], label='angular vel x', linewidth=2)
plt.plot(df['time'], df['ang_vel_y'], label='angular vel y', linewidth=2)
plt.plot(df['time'], df['ang_vel_z'], label='angular vel z', linewidth=2)
plt.plot(df['time'], df['project_gravity_x'], label='project gravity x', linewidth=2)
plt.plot(df['time'], df['project_gravity_y'], label='project gravity y', linewidth=2)
plt.plot(df['time'], df['project_gravity_z'], label='project gravity z', linewidth=2)

# plt.scatter(df['time'], df['ang_vel_x'], label='angular vel x', s=20, alpha=0.8)
# plt.scatter(df['time'], df['ang_vel_y'], label='angular vel y', s=20, alpha=0.8)
# plt.scatter(df['time'], df['ang_vel_z'], label='angular vel z', s=20, alpha=0.8)
# plt.scatter(df['time'], df['project_gravity_x'], label='project gravity x', s=20, alpha=0.8)
# plt.scatter(df['time'], df['project_gravity_y'], label='project gravity y', s=20, alpha=0.8)
# plt.scatter(df['time'], df['project_gravity_z'], label='project gravity z', s=20, alpha=0.8)

# plt.plot(df['time'], df['l_thigh_action'], label='l thigh action', linewidth=2)
# plt.plot(df['time'], df['l_calf_action'], label='l calf action', linewidth=2)
# plt.plot(df['time'], df['l_wheel_action'], label='l wheel action', linewidth=2, linestyle='--')
# plt.plot(df['time'], df['r_thigh_action'], label='r thigh action', linewidth=2)
# plt.plot(df['time'], df['r_calf_action'], label='r calf action', linewidth=2)
# plt.plot(df['time'], df['r_wheel_action'], label='r wheel action', linewidth=2, linestyle='--')

# plt.plot(df['time'], df['l_thigh_pos'], label='l thigh pos', linewidth=2)
# plt.plot(df['time'], df['l_calf_pos'], label='l calf pos', linewidth=2)
# plt.plot(df['time'], df['l_wheel_vel'], label='l wheel vel', linewidth=2)
# plt.plot(df['time'], df['r_thigh_pos'], label='r thigh pos', linewidth=2)
# plt.plot(df['time'], df['r_calf_pos'], label='r calf pos', linewidth=2)
# plt.plot(df['time'], df['r_wheel_vel'], label='r wheel vel', linewidth=2)
plt.xlabel('Time (s)')
plt.ylabel('')
plt.title('Obs state')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("imu_test.png")
plt.show()
