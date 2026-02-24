import pandas as pd
import matplotlib.pyplot as plt
import os

from ament_index_python.packages import get_package_prefix

# Get the full path to the package install directory
pkg_prefix = get_package_prefix('mbot_setpoint')
csv_path = os.path.join(pkg_prefix, 'lib', 'mbot_setpoint', 'logs', 'pid_debug_log.csv')

# Check if the file exists
if not os.path.exists(csv_path):
    raise FileNotFoundError(f"CSV log file not found at: {csv_path}")

# Load the CSV
df = pd.read_csv(csv_path)
df['time'] = df['time'] - df['time'].iloc[0]

print(df[['time', 'distance_error', 'angle_error', 'lin_cmd', 'ang_cmd']].head())
print("\nColumn stats:\n", df[['distance_error', 'angle_error', 'lin_cmd', 'ang_cmd']].describe())

# Plotting
plt.figure(figsize=(12, 6))

plt.subplot(2, 1, 1)
plt.plot(df['time'], df['distance_error'], label='Distance Error')
plt.plot(df['time'], df['angle_error'], label='Angle Error')
plt.ylabel('Error')
plt.title('PID Errors Over Time')
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(df['time'], df['lin_cmd'], label='Linear Velocity Cmd')
plt.plot(df['time'], df['ang_cmd'], label='Angular Velocity Cmd')
plt.xlabel('Time (s)')
plt.ylabel('Command')
plt.title('Velocity Commands Over Time')
plt.legend()

plt.tight_layout()
plt.show()