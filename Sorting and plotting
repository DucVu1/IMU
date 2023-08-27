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

# Sort Kalman angle and normal angle data column by column
sorted_kalman_data = [np.sort(data_array[:, col]) for col in range(num_columns // 2)]
sorted_normal_data = [np.sort(data_array[:, col + num_columns // 2]) for col in range(num_columns // 2)]

# Plot the sorted data for each column
for col_idx, (sorted_kalman_col_data, sorted_normal_col_data) in enumerate(zip(sorted_kalman_data, sorted_normal_data)):
    plt.plot(sorted_kalman_col_data, label=f"Kalman Column {col_idx + 1}")
    plt.plot(sorted_normal_col_data, label=f"Normal Column {col_idx + 1}", linestyle='dashed')

# Add labels and title
plt.xlabel('time')
plt.ylabel('Angle Value')
plt.title('Sorted Kalman Angle and Normal Angle Data for Each Column')
plt.legend()

# Show the plot
plt.show()
