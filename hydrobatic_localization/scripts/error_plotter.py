import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

sns.set_style("darkgrid")


df = pd.read_csv("state_estimator_log_without_mm.csv")

print(df.head())
df.columns = df.columns.str.strip()
print(df.columns.tolist())
df['error_x'] = df['est_pos_x'] - df['gt_pos_x']
df['error_y'] = df['est_pos_y'] - df['gt_pos_y']
df['error_z'] = df['est_pos_z'] - df['gt_pos_z']

# If you want to compute a composite error (e.g., Euclidean distance error)
df['error_total'] = (df['error_x']**2 + df['error_y']**2 + df['error_z']**2).apply(lambda x: x**0.5)

# Display basic statistics for error columns.
print(df[['error_x', 'error_y', 'error_z', 'error_total']].describe())


# Create a line plot of errors over time
plt.figure(figsize=(12, 6))
plt.plot(df['time'], df['error_x'], label='Error X', linewidth=2)
plt.plot(df['time'], df['error_y'], label='Error Y', linewidth=2)
plt.plot(df['time'], df['error_z'], label='Error Z', linewidth=2)
plt.xlabel('Time (s)', fontsize=12)
plt.ylabel('Error (m)', fontsize=12)
plt.title('State Estimator Errors Over Time', fontsize=14)
plt.legend(loc='upper right', fontsize=10)
plt.tight_layout()
plt.show()# Create a plot for the trajectories in the XY plane
plt.figure(figsize=(10, 8))

# Plot the estimated trajectory: X vs. Y
plt.plot(df['est_pos_x'], df['est_pos_y'], label='Estimated Trajectory',
         marker='o', markersize=3, linestyle='-', linewidth=1, alpha=0.8)

# Plot the ground truth trajectory: X vs. Y
plt.plot(df['gt_pos_x'], df['gt_pos_y'], label='Ground Truth Trajectory',
         marker='x', markersize=3, linestyle='--', linewidth=1, alpha=0.8)

# Adding labels and title
plt.xlabel("X Position (m)", fontsize=12)
plt.ylabel("Y Position (m)", fontsize=12)
plt.title("Trajectory Comparison in XY Plane", fontsize=14)
plt.legend(fontsize=10)
plt.grid(True)
plt.tight_layout()

# Display the plot
plt.show()