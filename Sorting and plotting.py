import numpy as np
import matplotlib.pyplot as plt

# Read data from the text file
with open("SWV_ITM_Data_Console.txt", 'r') as file:
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

# Plot the data for each column
for col_idx in range(num_columns // 2):
    plt.plot(-kalman_data[:, col_idx], label=f"Flipped Kalman Column {col_idx + 1}")
    plt.plot(normal_data[:, col_idx], label=f"Normal Column {col_idx + 1}", linestyle='dashed')

# Add labels and title
plt.xlabel('Time')
plt.ylabel('Angle Value')
plt.title('Flipped Kalman Angle and Normal Angle Data for Each Column')
plt.legend()

# Show the plot
plt.show()

