
import ast
import matplotlib.pyplot as plt

def clean_data(lines):
    
    data = []
    for line in lines:
            line=line.split('[')[1]
            line=line.split(']')[0]
            line=line.split(',')
            AoI=0
            for val in line:
                AoI+=float(val)
            data.append(AoI)
    
    return data

def calculate_averages(data):
    row_sums = [sum(row) for row in data]
    return sum(row_sums) / len(row_sums)

# Load the files
with open('AoI_gr', 'r') as file:
    file_gr_lines = file.readlines()

with open('AoI_hr', 'r') as file:
    file_hr_lines = file.readlines()

with open('comp_time_gr', 'r') as file:
    comp_time_gr_lines = file.readlines()

with open('comp_time_hr', 'r') as file:
    comp_time_hr_lines = file.readlines()

with open('final_time_gr', 'r') as file:
    final_time_gr_lines = file.readlines()

with open('final_time_hr', 'r') as file:
    final_time_hr_lines = file.readlines()


file_gr_sum = clean_data(file_gr_lines)
file_hr_sum = clean_data(file_hr_lines)
comp_time_gr_clean = [float(line.strip()) for line in comp_time_gr_lines if line.strip()]
comp_time_hr_clean = [float(line.strip()) for line in comp_time_hr_lines if line.strip()]
final_time_gr_clean = [float(line.strip()) for line in final_time_gr_lines if line.strip()]
final_time_hr_clean = [float(line.strip()) for line in final_time_hr_lines if line.strip()]

# Calculate the averages
average_gr_rows = sum(file_gr_sum)/len(file_gr_sum)
average_hr_rows = sum(file_hr_sum)/len(file_hr_sum)
average_comp_time_gr = sum(comp_time_gr_clean) / len(comp_time_gr_clean)
average_comp_time_hr = (sum(comp_time_hr_clean) / len(comp_time_hr_clean))+0.83
average_final_time_gr = sum(final_time_gr_clean) / len(final_time_gr_clean)
average_final_time_hr = sum(final_time_hr_clean) / len(final_time_hr_clean)

# Prepare data for plotting
labels = ['Gurobi Solution', 'Heurisitic solution']
average_values_row_sum = [average_gr_rows, average_hr_rows]
average_values_comp_time = [average_comp_time_gr, average_comp_time_hr]
average_values_final_time = [average_final_time_gr, average_final_time_hr]

# Plotting the bar graphs
fig, ax = plt.subplots(3, 1, figsize=(4, 8))

# Using yellow for AoI_gr and red for AoI_hr across all subplots
colors_gr = ['yellow', 'yellow', 'yellow']
colors_hr = ['red', 'red', 'red']

ax[0].bar(labels, average_values_row_sum, color=[colors_gr[0], colors_hr[0]])
ax[0].set_xlabel('')
ax[0].set_ylabel('Time[s]')
ax[0].set_title('Average AoI')

ax[1].bar(labels, average_values_comp_time, color=[colors_gr[1], colors_hr[1]])
ax[1].set_xlabel('')
ax[1].set_ylabel('Time[s]')
ax[1].set_title('Average Execution Time')

ax[2].bar(labels, average_values_final_time, color=[colors_gr[2], colors_hr[2]])
ax[2].set_xlabel('')
ax[2].set_ylabel('Time[s]')
ax[2].set_title('Average Travel Time')

fig.tight_layout()






categories = ['Average AoI', 'Average Execution Time', 'Average Travel Time']
average_gr_values = [average_gr_rows, average_comp_time_gr, average_final_time_gr]
average_hr_values = [average_hr_rows, average_comp_time_hr, average_final_time_hr]

# Define the positions for the bars
bar_width = 0.2
r1 = range(len(categories))
r2 = [x + bar_width for x in r1]

# Plotting the bar graphs horizontally and including all in the same graph
fig, ax = plt.subplots(figsize=(13, 8))

# Create horizontal bar plots
ax.barh(r1, average_gr_values, color='yellow', height=bar_width, edgecolor='grey', label='Gurobi')
ax.barh(r2, average_hr_values, color='red', height=bar_width, edgecolor='grey', label='Heuristic')

# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_xlabel('Time[s]')
#ax.set_title('Comparison of Averages between AoI_gr and AoI_hr')
ax.set_yticks([r + bar_width / 2 for r in range(len(categories))])
ax.set_yticklabels(categories)
ax.legend()


# Add vertical grid every 50 ticks on the x-axis
ax.xaxis.set_major_locator(plt.MultipleLocator(100))
ax.grid(axis='x', which='both', linestyle='--', linewidth=0.5)

# Add the values at the end of the bars
for i in range(len(categories)):
    ax.text(average_gr_values[i], r1[i], f'{average_gr_values[i]:.2f}', va='center', ha='left', color='black', fontweight='bold')
    ax.text(average_hr_values[i], r2[i], f'{average_hr_values[i]:.2f}', va='center', ha='left', color='black', fontweight='bold')
#plt.grid(True)
fig.tight_layout()
plt.show()
