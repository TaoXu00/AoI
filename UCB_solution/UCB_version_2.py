#file to implement the MAB UCB
import numpy as np 
import optimizer_guroby
import random
import matplotlib.pyplot as plt
from scipy.stats import norm
import math
import copy


    
def transmission_algorithm(b_real, b, visited_city_rescaled ):
    
    done=False
    AoI=[0 for _ in range(N)]
    data_memory=[0 for _ in range(N)]
    transmission_position=[0 for _ in range(N)]
    time_visit_list=[0]
    TX_data=[0 for _ in range(N)]
    
    city_index=1
    next_city=visited_city_rescaled[city_index]
    
    time=round(distance(sensor_positions[0],sensor_positions[next_city+1])/speed)
    time_visit_list.append(time)

    while not(done):
        
        current_city=next_city
        next_city=visited_city_rescaled[city_index+1]
        travel_time=round(distance(sensor_positions[current_city+1],sensor_positions[next_city+1])/speed)
        
        #update the data memeory
        data_memory[current_city]=1
        index_data=[]
        for i,el in enumerate(data_memory):
            if el==1:
                index_data.append(i)
        
        partial_sum_AoI=[]
        sum_AoI=[]
        
        
        for i in range(2**(len(index_data))):
            bin_index=integer_to_binary_list(i,len(index_data))
            
            temporary_AoI=copy.deepcopy(AoI)
            for j, el in enumerate(index_data):
                if bin_index[j]==0:
                    #0 means that I don't transmitt here-->travel_time to the next city + tx time
                    
                    temporary_AoI[el]= time + travel_time + round(DS[el]/ b[next_city],2)
                else:
                    
                    #1 means I can transmitt from the current location and so I compute the tx time
                    if b[current_city]>20:
                        temporary_AoI[el]= time + round(DS[el]/b[current_city],2)
                        
                    else:
                        #I set this becasue if the data rate is too low it doesn't have to transmitt
                        temporary_AoI[el]=time+1000000
                
            partial_sum_AoI.append(temporary_AoI)
            sum_AoI.append(sum(temporary_AoI))
           
        
        
        #capire come aggiornare l'indice e prendere la soluzione con l'aoi minore
        max_AoI=100000*N
        for index, aoi in enumerate(sum_AoI):
            #print(sum(aoi))
            if aoi<max_AoI:
                max_AoI=aoi
                index_max=index

        #update the various information vector
        index_max_bin=integer_to_binary_list(index_max, len(index_data))
        
        for j, el in enumerate(index_max_bin):
            if el==0:
                TX_data[index_data[j]]=0
            else:
                time = round(time + DS[index_data[j]]/b_real[current_city])
                transmission_position[index_data[j]]=current_city+1
                AoI[index_data[j]]=time
                TX_data[index_data[j]]=1
                data_memory[index_data[j]]=0
                
        
        time_visit_list.append(time) 
        
        time+=travel_time 
        time_visit_list.append(time)
        for i in range(len(AoI)):
            if TX_data[i]==0:
                #update the the AoI with the arrival time in the next city
                AoI[i]=time
        
        if next_city==visited_city_rescaled[-1]:
            #torno alla base
            current_city=next_city
            data_memory[visited_city_rescaled[-1]]=1
            next_city=0
            done=True
        else:
            city_index+=1

    #########################################################################
    
    travel_time=round(distance(sensor_positions[current_city+1], (0,0))/speed)
    
    index_data=[]
    for i,el in enumerate(data_memory):
        if el==1:
            index_data.append(i)
    
    partial_sum_AoI=[]
    sum_AoI=[]
    
    for i in range(2**(len(index_data))):
        bin_index=integer_to_binary_list(i,len(index_data))
        temporary_AoI=copy.deepcopy(AoI)
        #print(bin_index)
        for j, el in enumerate(index_data):
            if bin_index[j]==0:
                #0 means that I don't transmitt here-->travel_time to the next city + tx time
                temporary_AoI[el]= time + travel_time + round(DS[el]/ 100000,2)
            else:
                #1 means I can transmitt from the current location and so I compute the tx time
                if b[current_city]>20:
                    temporary_AoI[el]= time + round(DS[el]/b[current_city],2)
                else:
                    #I set this becasue if the data rate is too low it doesn't have to transmitt
                    temporary_AoI[el]=time+1000000
        
        partial_sum_AoI.append(temporary_AoI)
        sum_AoI.append(sum(temporary_AoI))
    
    #capire come aggiornare l'indice e prendere la soluzione con l'aoi minore
    max_AoI=100000*N
    for index, aoi in enumerate(sum_AoI):
        
        if aoi<max_AoI:
            max_AoI=aoi
            index_max=index

    #update the various information vector
    index_max_bin=integer_to_binary_list(index_max, len(index_data))
    for j, el in enumerate(index_max_bin):
        if el==0:
            TX_data[index_data[j]]=0
        else:
            time = round(time + DS[index_data[j]]/b_real[current_city])
            transmission_position[index_data[j]]=current_city+1
            AoI[index_data[j]]=time
            TX_data[index_data[j]]=1
            data_memory[index_data[j]]=0
    
    time_visit_list.append(time) 
        
    
    #########################################################################
    time= time + travel_time 
    time_visit_list.append(time) 
    for i in range(len(AoI)):
            if TX_data[i]==0:
                #Update the the AoI fot the data that have not been trasmitted. The AoI of these data corresponds to the time of arrival at the station base
                AoI[i]=time 

    
    
    
    _ , list_tx_reorder, time_spent=_data_manipulation( transmission_position, visited_city_rescaled, time_visit_list )
    
    return AoI, list_tx_reorder, time_spent, transmission_position

def distance(sensor1, sensor2):
    x1, y1 = sensor1
    x2, y2 = sensor2
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

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

def genera_valore_gaussiano(mean, standard_deviation):
    """
    Genera un valore casuale seguendo una distribuzione gaussiana (normale).
    
    :param media: La media della distribuzione gaussiana (default 0).
    :param deviazione_standard: La deviazione standard della distribuzione gaussiana (default 1).
    :return: Un valore casuale seguendo la distribuzione gaussiana specificata.
    """
    return random.gauss(mean, standard_deviation)

def plot_gaussian(media ,deviazione_standard):
    
    num_valori = 5000
    valori_gaussiani = [genera_valore_gaussiano(media, deviazione_standard) for _ in range(num_valori)]

    # Plot dei valori generati
    plt.figure(figsize=(10, 6))
    plt.hist(valori_gaussiani, bins=30, density=True, alpha=0.6, color='g')

    # Calcola e traccia la curva della distribuzione gaussiana teorica
    xmin, xmax = plt.xlim()
    x = np.linspace(xmin, xmax, 100)
    p = norm.pdf(x, media, deviazione_standard)
    plt.plot(x, p, 'k', linewidth=2)

    title = "Istogramma dei valori generati e curva della distribuzione gaussiana"
    plt.title(title)
    plt.xlabel('Valore')
    plt.ylabel('Densità di probabilità')
    plt.show()

def integer_to_binary_list(n,max):

    binary_string = bin(n)[2:]  # Convert integer to binary string
    binary_string = binary_string.zfill(max)
    binary_list = [int(digit) for digit in binary_string]  # Convert each digit to integer and store in list
    return binary_list

def _data_manipulation( transmission_position, visitied_sensor, time_list ):
        tx_base_station=[]
        list_tx_reorder=[]
        print(time_list)
        #change the structure of the transmission_position list
        for i in range(N+1):
            tx_prov=[]
            for index,tx in enumerate(transmission_position):
                if tx==i:
                    if i==0:
                        tx_base_station.append(index+1)
                    else:
                        tx_prov.append((index+1))
            if i>0:            
                list_tx_reorder.append(tx_prov)
        
        list_tx_reorder.append(tx_base_station)

        visitied_cities_rescaled=[cities+1 for cities in visitied_sensor[:-1] ]
        visitied_cities_rescaled.insert(0,0)
        
        time_spent=[str(round(time_list[i],1))+"-->"+str(round(time_list[i+1],1)) for i in range(1,len(time_list)-1,2) ]
        time_spent.insert(0,str(round(time_list[0],1))+"-->"+str(round(time_list[-1],1)))
        time_spent=time_spent[:]

        return  visitied_cities_rescaled, list_tx_reorder, time_spent


#define number of sensor
#one of the sensor is the base station
M=7
N=M-1
speed=7 #[m/s]

upper_bound = (500, 500)  # Limit coordinates (x_max, y_max)
lower_bound=(-500,-500)


N_trials_path=5
N_trials_transmission=10
c=50
#information to extract the bitrate matrix
matrix_width = 10
matrix_height = 10
min_x = -500
max_x = 500
min_y = -500
max_y = 500

#import coordinate from the file
#filename="../RL/DRL/DRL-solution/CoordinateFile"

path="./FolderFIle/"
name="CoordinateSetN_"+str(15)
sensor_positions=read_coordinates_from_file_with_bracket(path+name)
#print(sensor_positions)

#extract information about the real bitrate
file_path_mean= "../Bitrate_map/my_matrix_bitrate_mean.txt"
bit_rate_list=read_bitrate_from_file(file_path_mean,sensor_positions[:] )

file_path_std= "../Bitrate_map/my_matrix_bitrate_std.txt"
standard_dev=read_bitrate_from_file(file_path_std,sensor_positions[:] )
#print(bit_rate_list)
#generate random data size
data_size=random_data_generator(M, 2000, 2000)


GR=optimizer_guroby.gurobiOptimezerSol(M, speed, sensor_positions[:M], bit_rate_list[:M], data_size[:M])
GR.initialize_env()
AoI, total_AoI, list_postion_send, time_spent, list_visited_sensor_ordered=GR.optimizer_complete()

print(list_visited_sensor_ordered)

GR.draw_line_between_points(1,list_visited_sensor_ordered,sensor_positions[:M],list_postion_send, time_spent, upper_bound, lower_bound, "Sol Optimal", 1000,sum(AoI),  bit_rate_list[:M] )
#visited_city_=[0, 6, 4, 1, 3, 5, 2, 7]
#AoI_=[76.0, 230.0, 115.0, 68.0, 174.0, 72.0, 415.0]
visited_city_= list_visited_sensor_ordered
AoI_= AoI

AoI_real=sum(AoI_)
AoI_estimated=[]

#number of data available

#reascale the city index between 0 and N
visited_city_rescaled=[city-1 for city in visited_city_]

#times the program is run
run=1
#At the first time the drone transmitt from every locations
d=[1 for _ in range(N)]
DS=data_size
b_hat_list = [[0 for _ in range(N)] for _ in range(N_trials_transmission*N_trials_path)]
first=True

for j in range(N_trials_path):
    
    #During the first trip the drone tries to transmitt from every position
    if first:
        b_real=[]
        for j in range(N):
            value=genera_valore_gaussiano(bit_rate_list[j+1], standard_dev[j+1])
            if value > 125 :
                b_real.append(value)
            else:
                b_real.append(1)
        b_hat=[b_real[j] for j in range(N)]
        first=False
        

    b_hat_path=[]
    b_hat_path=copy.deepcopy(b_hat)
    b_hat_path.insert(0, 10000)
    GR=optimizer_guroby.gurobiOptimezerSol(M, speed, sensor_positions[:M], b_hat_path , data_size[:M])
    GR.initialize_env()
    AoI, total_AoI, list_postion_send, time_spent, list_visited_sensor_ordered=GR.optimizer_complete()
    GR.draw_line_between_points(1,list_visited_sensor_ordered,sensor_positions[:M],list_postion_send, time_spent, upper_bound, lower_bound, "Sol Optimal", 1000,sum(AoI),  bit_rate_list[:M] )
    
    visited_city_= list_visited_sensor_ordered
    visited_city_rescaled=[city-1 for city in visited_city_]
    print(visited_city_rescaled)
    for i in range(N_trials_transmission):
        
        b_real=[]
        for j in range(N):
            value=genera_valore_gaussiano(bit_rate_list[j+1], standard_dev[j+1])
            if value > 0 :
                b_real.append(value)
            else:
                b_real.append(1)
            
        
        b_mod=[]
        for j in range(N):
            value=b_hat[j] - c*math.sqrt(math.log(run)/d[j])
            if value > 0 :
                b_mod.append(value)
            else:
                b_mod.append(1)

        
        d_1=copy.deepcopy(d)
        #computing the tx order
        AoI, list_tx_reorder, time_spent2, transmission_position=transmission_algorithm(b_real ,b_mod, visited_city_rescaled)
        
        AoI_estimated.append(sum(AoI))

        for j in range(N):
            b_hat_list[run-1][j] = copy.deepcopy(b_hat[j])

        b_hat_old=copy.deepcopy(b_hat)
        for j in range(N):
            if j+1 in transmission_position:
                d[j]+=1
                b_hat[j]=(b_hat_old[j]*d_1[j] + b_real[j])/d[j]
        run+=1
        print(run)



GR.draw_line_between_points(2,list_visited_sensor_ordered,sensor_positions[:M],list_tx_reorder, time_spent2, upper_bound, lower_bound, "Sol UCB", 1000,sum(AoI),  bit_rate_list[:M] )



loss_AoI_func=[AoI_estimated[i] - AoI_real for i in range(len(AoI_estimated))]
cumulative_errors = []
cumulative_sum = 0
for i, error in enumerate(loss_AoI_func):
    cumulative_sum += error/(i+1)
    cumulative_errors.append(cumulative_sum)

plt.figure(3)

plt.plot(cumulative_errors)  # marker='o' adds points on the line
plt.xlabel('Trials')
plt.ylabel('Loss')
plt.title('Loss function with c: ' + str(c) )
plt.grid(True)



for i in range(len(b_hat)):
    expeted_value=[bit_rate_list[i+1] for _ in range(run)]
    plot_func=[b_hat_list[j][i] for j in range(run-1)]
    plt.figure(i*100)
    plt.plot(plot_func, color='blue')  
    plt.plot(expeted_value, color='red')  
    plt.xlabel('b_hat')
    plt.ylabel('Loss')
    plt.title('B_hat '+str(i+1))
    plt.grid(True)
    
plt.show()

