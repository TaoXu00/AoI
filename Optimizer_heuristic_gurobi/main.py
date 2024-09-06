#file used to run all the solution at the solver at the same time
import numpy as np 
import optmizer_heuristic
import optimizer_guroby
import random
import matplotlib.pyplot as plt

import time

def swap_xy_tuples(tuple_list):
    return [(y, -x) for (x, y) in tuple_list]


def plot_sensor_positions(sensor_positions, upper_bound, lower_bound):
  """
  Plots the sensor positions on a matplotlib scatter plot.

  Args:
      sensor_positions: A list of tuples representing sensor positions (x, y).
      limit_coord: A tuple representing the limit coordinates (x_max, y_max).
  """
  
  x, y = zip(*sensor_positions)  # Unpack sensor positions into separate lists
  
  plt.figure(figsize=(7, 7))  # Set figure size
  plt.scatter(x[0], y[0], marker='*', color='green', alpha=1, label='Starting point')  # Plot sensors
  
  for i, ele in enumerate(sensor_positions[1:]):
        plt.scatter(ele[0], ele[1], marker='o', color='blue', alpha=1)  # Plot sensors
        plt.text(ele[0]+10, ele[1]+10, "S"+str(i+1))


  
  values = np.linspace(lower_bound, upper_bound, 11) 
  #print the row of the grid
  for el in values:  
    plt.plot( [lower_bound, upper_bound], [el,el], 'c--', linewidth=0.5)
    plt.plot([el,el], [lower_bound, upper_bound],  'c--', linewidth=0.5 )
  

  plt.xlim(lower_bound[0], upper_bound[0])  # Set x-axis limits
  plt.ylim(lower_bound[1], upper_bound[1])  # Set y-axis limits
  plt.xlabel('X-Coordinate')
  plt.ylabel('Y-Coordinate')
  #plt.title('Sensor Positions')
  plt.grid(False)
  plt.legend()
  plt.show()



def read_coordinates_from_file( filename):
        coordinates = []
        with open(filename, 'r') as file:
            for line in file:
                x, y = map(int, line.strip().split(','))
                coordinates.append((x, y))
        return coordinates


def read_coordinates_from_file_with_bracket( filename):
        coordinates = []
        with open(filename, 'r') as file:
            for line in file:
                prov=line.split('(')[1]
                line=prov.split(')')[0]
                x, y = map(int, line.strip().split(','))
                coordinates.append((x, y))
        return coordinates


def is_within_matrix( x, y, min_x, max_x, min_y, max_y, matrix_width, matrix_height):
    """
    Checks if a Cartesian coordinate falls within the bounds of a matrix.

    Args:
        x: X coordinate.
        y: Y coordinate.
        min_x: Minimum X coordinate of the matrix space.
        max_x: Maximum X coordinate of the matrix space.
        min_y: Minimum Y coordinate of the matrix space.
        max_y: Maximum Y coordinate of the matrix space.
        matrix_width: Width of the matrix.
        matrix_height: Height of the matrix.

    Returns:
        A tuple containing the cell index (cell_x, cell_y) or None if outside the matrix.
    """
    
    # Calculate cell index (ensure integer division)
    cell_x = int((x - min_x) / (max_x - min_x) * (matrix_width))
    cell_y = int((y - min_y) / (max_y - min_y) * (matrix_height))

    # Check if cell index is within valid range
    if 0 <= cell_x < matrix_width and 0 <= cell_y < matrix_height:
        return cell_x, cell_y
    else:
        return None
    

def random_data_generator(total_sensor_number, lower_bound, upper_bound):
  
  data_random_list=[]
  for _ in range(total_sensor_number):
    data_random = random.uniform(lower_bound, upper_bound)
    data_random_list.append(int(data_random))
  
  return data_random_list


def read_bitrate_from_file(file_path: str, sensor_position: list ):
    delimiter = ","
# Read the matrix from the text file
    with open(file_path, "r") as file:
        # Read lines into a list
        lines = file.readlines()

    # Convert each line to a list of numbers (float by default)
    bitrate_matrix = np.array([list(map(float, line.strip().split(delimiter))) for line in lines])

    bitrate_coordinate=[]
    for coor in sensor_positions:
        cell_index = is_within_matrix(coor[0], coor[1] , min_x, max_x, min_y, max_y, matrix_width, matrix_height)  
        
        if bitrate_matrix[cell_index]==0:
            bitrate_coordinate.append(1)
        else:
            bitrate_coordinate.append(int(bitrate_matrix[cell_index]))

    bitrate_coordinate[0]=10000

    return bitrate_coordinate


def save_AoI_to_file(AoI, filename):
    """
    Save a list to a file.

    Args:
    my_list (list): The list to be saved.
    filename (str): The name of the file where the list will be saved.
    """
    with open(filename, 'a') as file:
        file.write(str(AoI)+"\n")

#Define the total number of sensor + Base station
M=8

#Speed of the drone
speed=7 #[m/s]

#information needed to extract the bitrate matrix
matrix_width = 10
matrix_height = 10
min_x = -500
max_x = 500
min_y = -500
max_y = 500

upper_bound = (500, 500) 
lower_bound=(-500,-500)

#generate random data size
data_size=random_data_generator(M, 800, 800)

Map=26  #15
Number_of_loop=5
#This for loop is used to cycle through the different file with the cooridnates
for i in range( Map, Map+Number_of_loop):

    #Read the correct file with a set of random coordinates
    path="./FolderFIle/"
    name="CoordinateSetN_" + str(i)
    sensor_positions=read_coordinates_from_file_with_bracket(path+name)
    print(sensor_positions)
    sensor_position_swap=swap_xy_tuples(sensor_positions)    

    #Once the coordiantes have been read they can be used to extract the information about the bitrate for every position
    file_path= "../Bitrate_map/my_matrix_bitrate_mean.txt"
    bit_rate_list = read_bitrate_from_file(file_path,sensor_positions[:])
    
    print(bit_rate_list)
    
    #plot_sensor_positions(sensor_position_swap[:M], upper_bound, lower_bound)
    
    ###### It is possible to compute the Solution using either gurobi or the heurisitic algorithm #########
    ###### If algorithm='HR' the solution will be optimized using the heuristic, if algorithm='GR' using Gurobi, if algorithm='BOTH' it compute the solution with both the optimizer ########
    ###### If plot=True the path will be plotted in a figure, otherwise no #######
    algorithm='BOTH'
    plot=True

    if algorithm=='GR':
        start_time_gr=time.time()
        GR=optimizer_guroby.gurobiOptimezerSol(M, speed, sensor_positions[:M], bit_rate_list[:M], data_size[:M])
        GR.initialize_env()
        AoI, total_AoI, list_postion_send, time_spent, list_visited_sensor_ordered=GR.optimizer_complete()
        end_time_gr=time.time()
        if plot:
            GR.draw_line_between_points(i,list_visited_sensor_ordered,sensor_position_swap[:M],list_postion_send, time_spent, upper_bound, lower_bound, " ", 0,sum(AoI),  bit_rate_list[:M] )
    
    elif algorithm=='HR':
        HR=optmizer_heuristic.HeuristicSolution(M, speed, sensor_positions[:M], bit_rate_list[:M], data_size[:M])
        #AoI_hr, list_visited_sensor_ordered_hr, list_postion_send_hr, time_spent_hr= HR.optimizer_compute()
        AoI_hr, list_visited_sensor_ordered_hr, list_postion_send_hr, time_spent_hr=HR.optimizer_compute_2()
        if plot:
            HR.draw_line_between_points(i,list_visited_sensor_ordered_hr,sensor_position_swap[:M],list_postion_send_hr, time_spent_hr, upper_bound, lower_bound, "Heuristic solution number"+ str(i), 0, round(sum(AoI_hr),1),  bit_rate_list[:M] )
    
    elif algorithm=='BOTH':
        start_time_gr=time.time()
        GR=optimizer_guroby.gurobiOptimezerSol(M, speed, sensor_positions[:M], bit_rate_list[:M], data_size[:M])
        GR.initialize_env()
        AoI, total_AoI, list_postion_send, time_spent, list_visited_sensor_ordered=GR.optimizer_complete()
        end_time_gr=time.time()

        start_time_hr=time.time()
        HR=optmizer_heuristic.HeuristicSolution(M, speed, sensor_positions[:M], bit_rate_list[:M], data_size[:M])
        #AoI_hr, list_visited_sensor_ordered_hr, list_postion_send_hr, time_spent_hr= HR.optimizer_compute()
        AoI_hr, list_visited_sensor_ordered_hr, list_postion_send_hr, time_spent_hr=HR.optimizer_compute_2()
        print(time_spent_hr)
        end_time_hr=time.time()
        
        if plot:
            GR.draw_line_between_points(i,list_visited_sensor_ordered,sensor_position_swap[:M],list_postion_send, time_spent, upper_bound, lower_bound, "Gurobi", 0,sum(AoI),  bit_rate_list[:M] )
            HR.draw_line_between_points(10*i,list_visited_sensor_ordered_hr,sensor_position_swap[:M],list_postion_send_hr, time_spent_hr, upper_bound, lower_bound, "Heuristic", 0, round(sum(AoI_hr),1),  bit_rate_list[:M] )

    
    #I save the results in files
    # save_AoI_to_file(AoI_hr, "AoI_hr")
    # save_AoI_to_file(AoI, "AoI_gr")
    # save_AoI_to_file(time_spent_hr[0].split(">")[1], "final_time_hr")
    # save_AoI_to_file(time_spent[0].split("-")[0], "final_time_gr")
    # save_AoI_to_file(end_time_hr-start_time_hr, "comp_time_hr")
    # save_AoI_to_file(end_time_gr - start_time_gr, "comp_time_gr")


plt.show()