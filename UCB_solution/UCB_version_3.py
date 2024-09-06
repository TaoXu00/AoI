#file to implement the MAB UCB
import numpy as np
import random
import matplotlib.pyplot as plt
from scipy.stats import norm
import math
import copy

class UCB3:
    def __init__(self, config):
        self.num_sensor=config['UCB3']['num_sensor'] #including the basestation, the first one is the base station
        self.speed = config['Drone']['drone_spped']  # 5[m/s]
        self.map_upper_bound = config['UCB3']['map_upper_bound']  # Limit coordinates (x_max, y_max)(500, 500)
        self.map_lower_bound =config['UCB3']['map_lower_bound'] #(-500, -500)
        self.N_trials_path=config['UCB3']['N_trials_path'] #50
        self.N_trials_transmission= config['UCB3']['N_trials_transmission'], #10
        self.c = config['UCB3']['c'] #15
        # information to extract the bitrate matrix
        self.matrix_width = config['UCB3']['matrix_width'] #10
        self.matrix_height = config['UCB3']['matrix_height'] #10
        self.min_x = config['UCB3']['min_x'] #-500
        self.max_x = config['UCB3']['max_x'] # 500
        self.min_y = config['UCB3']['min_y'] #-500
        self.max_y = config['UCB3']['max_y'] #500


    def __init__(self, num_sensor, drone_spped, map_upper_bound, map_lower_bound,N_trials_path, N_trials_transmission, c, matrix_width,matrix_height,min_x,max_x, min_y, max_y, plotter):
        self.num_sensor=num_sensor #including the basestation, the first one is the base station
        self.speed = drone_spped  # 5[m/s]
        self.map_upper_bound = map_upper_bound  # Limit coordinates (x_max, y_max)(500, 500)
        self.map_lower_bound =map_lower_bound #(-500, -500)
        self.N_trials_path=N_trials_path #50
        self.N_trials_transmission= N_trials_transmission, #10
        self.c = c #15
        # information to extract the bitrate matrix
        self.matrix_width = matrix_width #10
        self.matrix_height = matrix_height #10
        self.min_x = min_x #-500
        self.max_x = max_x # 500
        self.min_y = min_y #-500
        self.max_y = max_y #500
        self.plotter=plotter

    def dx_dy(self, sensor1, sensor2):
        x1, y1 = sensor1
        x2, y2 = sensor2
        return x2 - x1, y2 - y1

    def draw_line_between_points(self, N,list_visited_sensor_ordered, sensor_positions,transmission_position, time_spent,  upper_bound, lower_bound, title, battery_usage, AoI, bitrate):
        #N=self.num_sensor - 1
        plt.figure(figsize=(8, 6))

        #plot the grid
        values = np.linspace(lower_bound, upper_bound, 11)
        #print the row of the grid
        for el in values:
            plt.plot( [lower_bound, upper_bound], [el,el], 'c--', linewidth=0.5)
            plt.plot([el,el], [lower_bound, upper_bound],  'c--', linewidth=0.5 )


        plt.scatter(sensor_positions[0][0], sensor_positions[0][1],s=[200], marker="*", color='green', alpha=1 , label='Starting point')
        plt.text(sensor_positions[0][0]+10, sensor_positions[0][1]+10, "Start")
        plt.text(sensor_positions[0][0]+10, sensor_positions[0][1]+35, "t="+str(0.0))


        sens1=0
        for i, sensor in enumerate(list_visited_sensor_ordered[1:]):
            sens2=sensor
            x1, y1=sensor_positions[sens1]
            x2, y2=sensor_positions[sens2]
            #plt.plot([x1, x2], [y1, y2], color='red')
            dx, dy=self.dx_dy(sensor_positions[sens1], sensor_positions[sens2])
            plt.arrow(x1, y1, dx, dy, width = 1, color="red",length_includes_head=True, head_length=30, head_width=15)
            #modificato per prova
            plt.text(x2+10, y2+35, time_spent[i+1])
            #plt.text(x2+10, y2+35, time_spent[i])

            sens1=sens2
            #to print the last arrow


        dx, dy=self.dx_dy(sensor_positions[list_visited_sensor_ordered[-1]], sensor_positions[0])
        plt.arrow(sensor_positions[list_visited_sensor_ordered[-1]][0], sensor_positions[list_visited_sensor_ordered[-1]][1],
                    dx, dy, width = 1, color="red",length_includes_head=True, head_length=30, head_width=10)

        # Simple conversion to projected coordinates (example using linear scaling)
        for i, ele in enumerate(sensor_positions[1:]):
            plt.scatter(ele[0], ele[1], marker='o', color='blue', alpha=1)  # Plot sensors
            plt.text(ele[0]+10, ele[1]+10, "m"+str(i+1))
            plt.text(520, 490-(i*35),"m"+str(i+1)+": "+str(bitrate[i+1])+" B\s", color='red')



        #position of transmission plot
        for i,ele in enumerate(list_visited_sensor_ordered[1:]):
            if transmission_position[ele-1]:
                plt.text(520, 0-(i*35),"Sensor m"+str(ele)+": "+str(transmission_position[ele-1]))
            else:
                plt.text(520, 0-(i*35), "Sensor m"+str(ele)+": "+"NaN")

        if len(transmission_position)==N+1:
            plt.text(520, 0-((N)*35),"Base Station: "+str(transmission_position[N]))
        else:
            plt.text(520, 0-((N)*35),"Base Station: NaN")

        description="Battery: "+ str(battery_usage)+ "   AoI: "+ str(AoI)
        #plt.figtext(0.5,-0.01, description, ha='center', va='bottom', fontsize=20)  # Adjust position and size as needed
        plt.text(-400, -610, description, fontsize=20)
        #plt.xlabel('X-Coordinate')
        plt.ylabel('Y-Coordinate')
        plt.title(title)
        plt.xlim(lower_bound[0], upper_bound[0])  # Set x-axis limits
        plt.ylim(lower_bound[1], upper_bound[1])  # Set y-axis limits
        plt.grid(False)
        plt.legend()

    def transmission_algorithm(self, b_real, b, visited_sensor_rescaled, DS, sensor_positions):
        #b_real is the bite rate realization during that trip
        #b is the estimated bite rate
        N=self.num_sensor-1
        done=False
        AoI=[0 for _ in range(N)]
        data_memory=[0 for _ in range(N)]
        transmission_position=[0 for _ in range(N)]
        time_visit_list=[0]
        TX_data=[0 for _ in range(N)]

        actual_sensor_index=1
        next_sensor=visited_sensor_rescaled[actual_sensor_index]

        time=round(self.distance(sensor_positions[0],sensor_positions[next_sensor+1])/self.speed)
        time_visit_list.append(time)

        while not(done):
            current_sensor=next_sensor
            next_sensor=visited_sensor_rescaled[actual_sensor_index+1]
            travel_time=round(self.distance(sensor_positions[current_sensor+1],sensor_positions[next_sensor+1])/self.speed)

            #update the data memeory
            data_memory[current_sensor]=1
            index_data=[]
            for i, data in enumerate(data_memory):
                if data==1:
                    index_data.append(i)

            sum_AoI=[]
            temp_time=0

            for i in range(2**(len(index_data))):
                bin_index=self.integer_to_binary_list(i,len(index_data))
                temporary_AoI=copy.deepcopy(AoI)
                for j, index in enumerate(index_data):
                    if bin_index[j]==0:
                        #0 means that I don't transmitt here-->travel_time to the next city + tx time
                        temporary_AoI[index]= time + travel_time + round(DS[index]/ b[next_sensor],2)
                    else:
                        #1 means I can transmitt from the current location and so I compute the tx time
                        if b[current_sensor]>10:
                            temporary_AoI[index]= time + round(DS[index]/b[current_sensor],2)
                        else:
                            #I set this becasue if the data rate is too low it doesn't have to transmitt
                            temporary_AoI[index]=time+1000000

                sum_AoI.append(sum(temporary_AoI))


            min_AoI=100000*N
            for index, aoi in enumerate(sum_AoI):
                #print(sum(aoi))
                if aoi<min_AoI:
                    min_AoI=aoi
                    index_min=index

            index_min_bin=self.integer_to_binary_list(index_min, len(index_data))

            for j, num in enumerate(index_min_bin):
                #Even if the drone need to transmitt, it need to check if the connection is present
                if num==0:
                    TX_data[index_data[j]]=0

                # elif num==1 and b_real[current_sensor]<50:
                #     TX_data[index_data[j]]=0
                #     #if the heuristic algorithm said to tx from a certain position but there's no connection,
                #     #the drone
                #     time = round(time + 30)
                else:
                    time = round(time + DS[index_data[j]]/b_real[current_sensor])
                    transmission_position[index_data[j]]=current_sensor+1
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

            if next_sensor==visited_sensor_rescaled[-1]:
                #torno alla base
                current_sensor=next_sensor
                data_memory[visited_sensor_rescaled[-1]]=1
                next_sensor=0
                done=True
            else:
                actual_sensor_index+=1

        #########################################################################
        #This section is used to consider the base station
        travel_time=round(self.distance(sensor_positions[current_sensor+1], (0,0))/self.speed)
        index_data=[]
        for i,data in enumerate(data_memory):
            if data==1:
                index_data.append(i)


        sum_AoI=[]
        for i in range(2**(len(index_data))):
            bin_index=self.integer_to_binary_list(i,len(index_data))
            temporary_AoI=copy.deepcopy(AoI)
            #print(bin_index)
            for j, index in enumerate(index_data):
                if bin_index[j]==0:
                    #0 means that I don't transmitt here-->travel_time to the next city + tx time
                    temporary_AoI[index]= time + travel_time + round(DS[index]/ 100000,2)
                else:
                    #1 means I can transmitt from the current location and so I compute the tx time
                    if b[current_sensor]>20:
                        temporary_AoI[index]= time + round(DS[index]/b[current_sensor],2)
                    else:
                        #I set this becasue if the data rate is too low it doesn't have to transmitt
                        temporary_AoI[index]=time+1000000

            sum_AoI.append(sum(temporary_AoI))


        min_AoI=100000*N
        for index, aoi in enumerate(sum_AoI):
            if aoi<min_AoI:
                min_AoI=aoi
                index_min=index

        index_min_bin=self.integer_to_binary_list(index_min, len(index_data))
        for j, num in enumerate(index_min_bin):
            if num==0:
                TX_data[index_data[j]]=0
            else:
                time = round(time + DS[index_data[j]]/b_real[current_sensor])
                transmission_position[index_data[j]]=current_sensor+1
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


        _ , list_tx_reorder, time_spent=self._data_manipulation( transmission_position, visited_sensor_rescaled, time_visit_list )

        return AoI, list_tx_reorder, time_spent, transmission_position

    def path_planning_heuristics(self, DR, coordinates):
        current_location=(0,0)
        visited_sensors=[-1]
        #print(DR)
        for _ in range(len(coordinates)-1):
            #Compute the distances between the current position and all the ohters sensors
            D=[]
            for pos in coordinates[1:]:
                D.append(self.distance(current_location, pos))


            #Compute the weights W in order to decide the next sensor
            W=[]
            for i in range(len(D)):
                if i in visited_sensors:
                    W.append(1000000000000)
                else:
                    # if DR[i]==1:
                    #     W.append(round(D[i]/(1),4))
                    # else:
                    #      W.append(round(D[i]/(math.log10(DR[i])),4))
                    #normalize the two factors
                    if DR[i]==1:
                        n_DR=0.8
                    else:
                        n_DR=DR[i]/300
                    n_D=D[i]/702
                    W.append(round(n_D/n_DR,4))
            #The next sensor is the one with the lowest weight
            next_sensor_index = W.index(min(W))
            visited_sensors.append(next_sensor_index)
            print(f'visited_sensors: {[s+1 for s  in visited_sensors ]}')
            print(f'weight list: {W}')
            current_location=coordinates[next_sensor_index+1]

        return visited_sensors

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

    def is_within_matrix(self, x, y, min_x, max_x, min_y, max_y, matrix_width, matrix_height):
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

    def random_data_generator(self, lower_bound, upper_bound):

        data_random_list=[]
        for _ in range(self.num_sensor):
            data_random = random.uniform(lower_bound, upper_bound)
            data_random_list.append(int(data_random))

        return data_random_list

    def read_bitrate_from_file(self,file_path: str, sensor_positions: list):
        delimiter = ","
    # Read the matrix from the text file
        with open(file_path, "r") as file:
            # Read lines into a list
            lines = file.readlines()

        # Convert each line to a list of numbers (float by default)
        bitrate_matrix = np.array([list(map(float, line.strip().split(delimiter))) for line in lines])

        bitrate_coordinate=[]
        for coor in sensor_positions:
            cell_index = self.is_within_matrix(coor[0], coor[1] , self.min_x, self.max_x, self.min_y, self.max_y, self.matrix_width, self.matrix_height)

            if bitrate_matrix[cell_index]<123: # in case there is no connection
                bitrate_coordinate.append(1)
            else:
                bitrate_coordinate.append(int(bitrate_matrix[cell_index]))

        bitrate_coordinate[0]=10000 #this is for the base station.

        return bitrate_coordinate

    def read_std_from_file(self,file_path: str, sensor_positions: list):
        delimiter = ","
    # Read the matrix from the text file
        with open(file_path, "r") as file:
            # Read lines into a list
            lines = file.readlines()

        # Convert each line to a list of numbers (float by default)
        bitrate_matrix = np.array([list(map(float, line.strip().split(delimiter))) for line in lines])

        bitrate_coordinate=[]
        for coor in sensor_positions:
            cell_index = self.is_within_matrix(coor[0], coor[1] , self.min_x, self.max_x, self.min_y, self.max_y, self.matrix_width, self.matrix_height)

            bitrate_coordinate.append(int(bitrate_matrix[cell_index]))


        bitrate_coordinate[0]=10000

        return bitrate_coordinate

    def save_AoI_to_file(self, AoI, filename):
        """
        Save a list to a file.

        Args:
        my_list (list): The list to be saved.
        filename (str): The name of the file where the list will be saved.
        """
        with open(filename, 'a') as file:
            file.write(str(AoI)+"\n")

    def genera_valore_gaussiano(self,mean, standard_deviation):
        """
        Genera un valore casuale seguendo una distribuzione gaussiana (normale).

        :param media: La media della distribuzione gaussiana (default 0).
        :param deviazione_standard: La deviazione standard della distribuzione gaussiana (default 1).
        :return: Un valore casuale seguendo la distribuzione gaussiana specificata.
        """
        return random.gauss(mean, standard_deviation)

    def plot_gaussian(self,media ,deviazione_standard):

        num_valori = 5000
        valori_gaussiani = [self.genera_valore_gaussiano(media, deviazione_standard) for _ in range(num_valori)]

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

    def integer_to_binary_list(self,n,max):

        binary_string = bin(n)[2:]  # Convert integer to binary string
        binary_string = binary_string.zfill(max)
        binary_list = [int(digit) for digit in binary_string]  # Convert each digit to integer and store in list
        return binary_list

    def _data_manipulation(self, transmission_position, visitied_sensor, time_list ):
            tx_base_station=[]
            list_tx_reorder=[]
            #print(time_list)
            #change the structure of the transmission_position list
            #for i in range(N+1):
            for i in range (self.num_sensor):
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

    def reference_solution(self, coordinates, actual_DR, DS, sensor_positions):

        _visited_sensor_rescaled=self.path_planning_heuristics(actual_DR, coordinates)
        AoI, list_tx_reorder, time_spent, transmission_position_ref=self.transmission_algorithm(actual_DR, actual_DR, _visited_sensor_rescaled, DS, sensor_positions)
        trajectory_ref=[position+1 for position in _visited_sensor_rescaled]
        actual_DR.insert(0,100000)
        self.draw_line_between_points(self.num_sensor-1 ,trajectory_ref, sensor_positions[:self.num_sensor],list_tx_reorder, time_spent,  self.upper_bound, self.lower_bound, "Reference Solution", 1, sum(AoI), actual_DR)

        return AoI, trajectory_ref, transmission_position_ref

    def sensor_map_and_bite_rate_generation(self, sensor_map_direct, sensor_map_name, biterate_mean_file_path, bitrate_std_file_path):
        # import coordinate from the file
        #sensor_map_direct = "./sensor_maps/"  # this is the maps
        #sensor_map_name = "CoordinateSetN_" + str(59)  # there are 39 sensors in this map
        #biterate_file_path= "../Bitrate_map/my_matrix_bitrate_mean.txt"
        #file_path_std = "../Bitrate_map/my_matrix_bitrate_std.txt"
        sensor_positions = self.read_coordinates_from_file_with_bracket(sensor_map_direct + sensor_map_name)  # the sensor positions includes the base station (the first element) with (0,0)
        # print(sensor_positions)
        # extract information about the real bitrate

        bit_rate_list = self.read_bitrate_from_file(biterate_mean_file_path, sensor_positions[:])
        standard_dev = self.read_std_from_file(bitrate_std_file_path,sensor_positions[:])
        # generate random data size
        data_size = self.random_data_generator(self.num_sensor, 2000, 2000)  # it seems we use different data size for each sensor
        return sensor_positions, bit_rate_list, standard_dev, data_size

    def run_UCB(self):
        #define number of sensor
        #one of the sensor is the base station
        N=self.num_sensor-1
        sensor_positions, bit_rate_list, standard_dev, data_size = self.sensor_map_and_bite_rate_generation(self.sensor_map_direct, self.sensor_map_name, self.biterate_mean_file_path, self.bitrate_std_file_path)
        #compute the solution with the known knowledge
        AoI_ref, trajecotry_ref, transmission_position_ref=self.reference_solution(sensor_positions[:self.num_sensor], bit_rate_list[1:self.num_sensor], data_size, sensor_positions) # this is calculated based on the real data rate
        AoI_sum_ref=sum(AoI_ref)
        print("##### Reference trajecotry, transmission decision and the sum AoI: ####")
        print(trajecotry_ref)
        print(transmission_position_ref)
        print(AoI_sum_ref)

        AoI_estimated=[]
        #times the program is run
        run=1
        #At the first time the drone transmitt from every locations
        d=[1 for _ in range(N)]
        b_hat_list = [[0 for _ in range(N)] for _ in range(self.N_trials_transmission * self.N_trials_path)] #every 50 trails, we update the path.
        first=True
        list_learned_trajectory=[]
        b_real_list=[]
        image=1
        b_mod_list=[]
        learned_trajectory_list=[]
        for j in range(self.N_trials_path):
            #During the first trip the drone tries to transmitt from every position
            if first:
                b_real=[]
                for j in range(N):
                    value=self.genera_valore_gaussiano(bit_rate_list[j+1], standard_dev[j+1])
                    if value > 10 and value <= 300 :
                        b_real.append(value)
                    elif value > 300 :
                        b_real.append(300)
                    else:
                        b_real.append(1)
                b_hat=[b_real[j] for j in range(N)]
                first=False
                b_real_list.append(b_real)

            b_hat_path=[]
            b_hat_path=copy.deepcopy(b_hat) #this is used for calculating the path with weighted heuristic techniques
            #b_hat_path.insert(0, 10000)

            visited_sensor_rescaled=self.path_planning_heuristics(b_hat_path, sensor_positions[:self.num_sensor]) # trajectory of the calculated trajectory
            list_visited_sensor_ordered = [sensor + 1 for sensor in visited_sensor_rescaled]
            list_learned_trajectory.append(list_visited_sensor_ordered)

            for i in range(self.N_trials_transmission): # the computed path kept same within the N_trials_transmission, only the data transmission is learned by the UCB.
                print(run)
                #print(f'b_hat: {b_hat}')
                b_real=[]
                for j in range(N):
                    value=self.genera_valore_gaussiano(bit_rate_list[j+1], standard_dev[j+1])
                    if value > 10 and value <= 300 :
                        b_real.append(value)
                    elif value > 300 :
                        b_real.append(300)
                    else:
                        b_real.append(1)


                b_mod=[]
                for j in range(N):
                    value= b_hat[j] + self.c*math.sqrt(math.log(run)/d[j]) #the value used by the UCB policy, b_hat and d.
                    if value > 1 :
                        b_mod.append(value)
                    else:
                        b_mod.append(1)

                b_mod_list.append(b_mod)
                d_1=copy.deepcopy(d)
                #computing the tx order
                AoI, list_tx_reorder, time_spent, transmission_position=self.transmission_algorithm(b_real ,b_mod, visited_sensor_rescaled, data_size)
                AoI_estimated.append(sum(AoI))
                print(f"transmission_position: {transmission_position}")
                for j in range(N):
                    b_hat_list[run-1][j] = copy.deepcopy(b_hat[j])

                b_real_list.append(b_real)
                b_hat_old=copy.deepcopy(b_hat)
                for j in range(N):
                    if j+1 in transmission_position:
                        d[j]+=1
                        b_hat[j]=(b_hat_old[j]*d_1[j] + b_real[j])/d[j]
                run+=1

            # draw_line_between_points(image,list_visited_sensor_ordered, sensor_positions[:M],list_tx_reorder, time_spent,  upper_bound, lower_bound, "Various Path Solution"+str(image), 1, sum(AoI), bit_rate_list[:M])
            image+=1

        b_real_final=copy.deepcopy(b_real)
        b_real_final.insert(0, 10000)
        self.draw_line_between_points(self.num_sensor-1 ,list_visited_sensor_ordered, sensor_positions[:self.num_sensor],list_tx_reorder, time_spent,  self.upper_bound, self.lower_bound, "Final Solution", 1, sum(AoI), b_real_final)
        #AoI_estimated.insert(0, 25000 )



        plt.figure()
        instant_regret=[AoI_estimated[i] for i in range(len(AoI_estimated))]
        plt.plot(instant_regret)  # marker='o' adds points on the line
        plt.xlabel('Trials')
        plt.ylabel('Loss')
        plt.title('Instant AOI: ')
        plt.grid(True)

        #output infomation
        diff=0
        print("######## trajectory during learning: ######")
        #print(list_learned_trajectory)
        for i, trajecotry in enumerate(list_learned_trajectory):
            if trajecotry != trajecotry_ref:
                print(f'find a differnt trajectory: {trajecotry} at {i} iteration')
                diff=diff+1
        print(f'{diff} diff trajectories are found.')
        print("AoI_estimated:")
        print(AoI_estimated)

        self.plotter.plot_regret(AoI_estimated, AoI_sum_ref, self.c)

        self.plotter.plot_average_cumulative_AoI(run, AoI_estimated, AoI_sum_ref)

        self.plotter.plot_mean_error_b_hat(N, run, b_hat_list, bit_rate_list[1:self.num_sensor])

        window_sizes = [40, 100, 150, 200]
        #plot_file.plot_moving_avarage_AoI(run, AoI_estimated, AoI_real, window_sizes)

        #plot_file.plot_b_hat(run, b_hat, bit_rate_list, b_hat_list, b_real_list)

        #plot_file.plot_b_mod(run, b_hat, bit_rate_list, b_mod_list, b_hat_list)

        plt.show()


