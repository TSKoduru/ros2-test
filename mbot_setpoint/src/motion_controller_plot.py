import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV from logs directory
csv_path = "../logs/pid_debug_log.csv"

# Check if the file exists
import os
if not os.path.exists(csv_path):
    raise FileNotFoundError(f"CSV log file not found at: {csv_path}")

# Load the CSV
df = pd.read_csv(csv_path)
df['time'] = df['time'] - df['time'].iloc[0]

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

plt.savefig("../logs/pid_debug_plot.png", dpi=150, bbox_inches='tight')
