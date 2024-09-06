#plot the mean of the cumulative AOI
import pandas as pd
import matplotlib.pyplot as plt

# Read the file content
file_path = "cumulative_aoi.txt"  # Replace with your actual file path

with open(file_path, "r") as file:
    lines = file.readlines()

# Processing the lines to create a bidimensional list
bidimensional_list = [list(map(float, line.strip().split(', '))) for line in lines]

# Convert the bidimensional list to a DataFrame for easier processing
df = pd.DataFrame(bidimensional_list)

# Calculate the mean of each column
mean_vector = df.mean()
mean_vector_list = mean_vector.tolist()

# Calculate the median of each column
median_vector = df.median()
median_vector_list = median_vector.tolist()

# Plot the mean vector
plt.figure(figsize=(14, 7))
plt.plot(mean_vector_list, marker='o', linestyle='-', color='b', label='Mean')
plt.xlabel('Trials')
plt.ylabel('AoI')
plt.title('Mean of the Average cumulative AoI')
plt.legend()
plt.show()

# Plot the median vector
plt.figure(figsize=(14, 7))
plt.plot(median_vector_list, marker='o', linestyle='-', color='g', label='Median')
plt.xlabel('Index')
plt.ylabel('Value')
plt.title('Median of Each Column')
plt.legend()
plt.show()
