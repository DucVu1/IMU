import numpy as np
import matplotlib.pyplot as plt

# Read data from the text file
with open("test4.txt", 'r') as file:
    lines = file.readlines()

# Split and convert data to a 2D array
data = [list(map(float, line.split())) for line in lines]

# Convert the list of lists to a NumPy array
data_array = np.array(data)

# Get the number of columns in the data
num_columns = data_array.shape[1]

# Split the data into Kalman angle and normal angle arrays
kalman_data = data_array[:, :num_columns // 2]
normal_data = data_array[:, num_columns // 2:]

# Calculate statistics for Kalman angle data
kalman_mean = np.mean(kalman_data, axis=0)
kalman_variance = np.var(kalman_data, axis=0)
kalman_deviation = np.sqrt(kalman_variance)

# Calculate statistics for normal angle data
normal_mean = np.mean(normal_data, axis=0)
normal_variance = np.var(normal_data, axis=0)
normal_deviation = np.sqrt(normal_variance)

# Create a figure and axes
fig, ax = plt.subplots()

# Plot the data for each column
for col_idx in range(num_columns // 2):
    plt.plot(-kalman_data[:, col_idx], label=f"Flipped Kalman Column {col_idx + 1}")
    plt.plot(normal_data[:, col_idx], label=f"Normal Column {col_idx + 1}", linestyle='dashed')

# Draw mean lines
for col_idx in range(num_columns // 2):
    ax.axhline(y=kalman_mean[col_idx], color='red', linestyle='dotted', label=f"Kalman Mean: {kalman_mean[col_idx]:.2f}")
    ax.axhline(y=normal_mean[col_idx], color='blue', linestyle='dotted', label=f"Normal Mean: {normal_mean[col_idx]:.2f}")

# Draw variance and deviation areas
for col_idx in range(num_columns // 2):
    ax.fill_between(range(len(kalman_data)), kalman_mean[col_idx] - kalman_deviation[col_idx],
                    kalman_mean[col_idx] + kalman_deviation[col_idx], color='red', alpha=0.2,
                    label=f"Kalman Deviation: {kalman_deviation[col_idx]:.2f}")
    ax.fill_between(range(len(normal_data)), normal_mean[col_idx] - normal_deviation[col_idx],
                    normal_mean[col_idx] + normal_deviation[col_idx], color='blue', alpha=0.2,
                    label=f"Normal Deviation: {normal_deviation[col_idx]:.2f}")

# Add labels and title
plt.xlabel('Time')
plt.ylabel('Angle Value')
plt.title('Flipped Kalman Angle and Normal Angle Data for Each Column')
plt.legend()

# Show the plot
plt.show()
