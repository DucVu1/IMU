import numpy as np
import matplotlib.pyplot as plt

# Read data from the text file
with open("test.txt", 'r') as file:
    lines = file.readlines()

# Split and convert data to a 2D array
data = [list(map(float, line.split())) for line in lines]

# Convert the list of lists to a NumPy array
data_array = np.array(data)

# Get the number of columns in the data
num_columns = data_array.shape[1]

# Split the data into Kalman angle and normal angle arrays
kalman_data = data_array[:, :num_columns // 2]
kalman_mean = np.mean(kalman_data)
normal_data = data_array[:, num_columns // 2:]
normal_mean = np.mean(normal_data)
normal_data2 = data_array[:, :num_columns // 2]
normal_mean2 = np.mean(normal_data2)

# Create a figure and axes
fig, ax = plt.subplots()

# Plot the data for each column
for col_idx in range(num_columns // 2):
    plt.plot(-kalman_data[:, col_idx], label=f"first {col_idx + 1}")
    plt.plot(normal_data[:, col_idx], label=f"second {col_idx + 1}", linestyle='dashed')
    plt.plot(normal_data2[:, col_idx], label=f"third {col_idx + 1}", linestyle='dotted')

# Draw mean lines
ax.axhline(y=kalman_mean, color='red', linestyle='dotted', label=f"Kalman Mean: {kalman_mean:.2f}")
ax.axhline(y=normal_mean, color='blue', linestyle='dotted', label=f"Normal Mean: {normal_mean:.2f}")
ax.axhline(y=normal_mean2, color='green', linestyle='dotted', label=f"Normal Mean2: {normal_mean2:.2f}")

# Add labels and title
plt.xlabel('Time')
plt.ylabel('Angle Value')
plt.title('Flipped Kalman Angle and Normal Angle Data for Each Column')
plt.legend()

# Show the plot
plt.show()

