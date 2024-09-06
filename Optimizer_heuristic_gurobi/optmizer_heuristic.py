import matplotlib.pyplot as plt
import numpy as np
import math
import random
import copy

class HeuristicSolution:
    def __init__(self, N , v, coordinates , DataRate, DataSize):
        self.N = N-1
        self.v=v
        self.speed=v
        self.DR= DataRate[1:]
        self.DS=DataSize[1:] 
        self.coordinates=coordinates
        self.sensor_positions=coordinates
    
    def distance(self, sensor1, sensor2):
        x1, y1 = sensor1
        x2, y2 = sensor2
        return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

    def integer_to_binary_list(self, n,max):
        binary_string = bin(n)[2:]  # Convert integer to binary string
        binary_string = binary_string.zfill(max)
        binary_list = [int(digit) for digit in binary_string]  # Convert each digit to integer and store in list
        return binary_list

    def dx_dy(self, sensor1, sensor2):
        x1, y1 = sensor1
        x2, y2 = sensor2
        return x2 - x1, y2 - y1

    def draw_line_between_points(self,n,list_visited_sensor_ordered, sensor_positions,transmission_position, time_spent,  upper_bound, lower_bound, title, battery_usage, AoI, bitrate):

        plt.figure(n,figsize=(7, 7))
        
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
        plt.text(510,480,"DR[B\s]", color='red')
        for i, ele in enumerate(sensor_positions[1:]):
            plt.scatter(ele[0], ele[1], marker='o', color='blue', alpha=1)  # Plot sensors
            plt.text(ele[0]+10, ele[1]+10, "S"+str(i+1))
            #plt.text(520, 490-(i*35),"S"+str(i+1)+": "+str(bitrate[i+1])+" B\s", color='red')

            plt.text(510, 445-(i*35),"S"+str(i+1)+": "+str(bitrate[i+1]), color='red')
            
            if transmission_position[i]:
             plt.text(ele[0]+10, ele[1]+60,str(transmission_position[i]),color='green')
            else:
             plt.text(ele[0]+10, ele[1]+60, "NaN",color='green') 
            
        
        
        #position of transmission plot
        # for i,ele in enumerate(list_visited_sensor_ordered[1:]):
        #     if transmission_position[ele-1]:
        #         plt.text(520, 0-(i*35),"Sensor m"+str(ele)+": "+str(transmission_position[ele-1]))
        #     else:
        #         plt.text(520, 0-(i*35), "Sensor m"+str(ele)+": "+"NaN")

        if len(transmission_position)==self.N+1:
            plt.text( 0+10,0+60,str(transmission_position[self.N]), color='green')
        else:
            plt.text(0+10, 0+60,"NaN", color='green')     
        
        description="AoI: "+ str(AoI) 
        plt.figtext(0.5,-0.002, description, ha='center', va='bottom', fontsize=20)  
        #plt.figtext(0.5,-0.01, description, ha='center', va='bottom', fontsize=20)  # Adjust position and size as needed
        
        #plt.xlabel('X-Coordinate')
        plt.ylabel('Y-Coordinate')
        plt.title(title)
        plt.xlim(lower_bound[0], upper_bound[0])  # Set x-axis limits
        plt.ylim(lower_bound[1], upper_bound[1])  # Set y-axis limits
        plt.grid(False)
        plt.legend()


    def _data_manipulation(self, transmission_position, visitied_sensor, time_list ):
        tx_base_station=[]
        list_tx_reorder=[]
        #change the structure of the transmission_position list
        for i in range(self.N+1):
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
        
        time_spent=[str(round(time_list[i],1))+"-->"+str(round(time_list[i+1],1)) for i in range(1,len(time_list),2) ]
        time_spent.insert(0,str(round(time_list[0],1))+"-->"+str(round(time_list[-1],1)))
        time_spent=time_spent[:-1]

        return  visitied_cities_rescaled, list_tx_reorder, time_spent

    def optimizer_compute(self):

        #Initialization of the list used in the code
        done=False
        
        #Starting position of the drone
        current_location=(0,0)
        visitied_sensor=[]
        data_in_memory=[0 for _ in range(self.N)]
        AoI=[0 for _ in range(self.N)]
        TX_data=[0 for _ in range(self.N)]
        time=0
        transmission_position=[0 for _ in range(self.N)]
        time_list=[0]
        
        while not(done):
            
            if visitied_sensor:
                data_in_memory[visitied_sensor[-1]]=1
            
            #Compute the distances between the current position and all the ohters sensors
            D=[]
            for pos in self.coordinates[1:]:
                D.append(self.distance(current_location, pos))
            

            #Compute the weights W in order to decide the next sensor
            W=[]
            for i in range(len(D)):
                if i in visitied_sensor:
                    W.append(1000000000000)    
                else:
                    if self.DR[i]==1:
                        W.append(round(D[i]/(1),4))
                    else:
                        W.append(round(D[i]/(math.log10(self.DR[i])),4))


            #The next sensor is the one with the lowest weight
            next_sensor_index = W.index(min(W))
            travel_time=D[next_sensor_index]/self.v
            
            #Extract the index of the data in memory
            data_in_memory_index=[]
            for i,data in enumerate(data_in_memory):
                if data==1:
                    data_in_memory_index.append(i)
            
            #If the drone has visited at least one sensor it is possible to compute the transmission position
            if visitied_sensor:
               
                list_sum_temporary_AoI=[]
                
                for i in range(2**(len(data_in_memory_index))):
                    temporary_AoI=AoI
                    #extract the binary number that corresponds to the index i
                    bin_index=self.integer_to_binary_list(i,len(data_in_memory_index))
                   
                    #Using the binary number the transmission combination is computed
                    for j, data in enumerate(data_in_memory_index):
                        if bin_index[j]==0:
                            #0 means the data isn't transmitted in the current position-->the AoI include travel_time to the next city + tx time from the next location
                            temporary_AoI[data] = time + travel_time + round(self.DS[data]/self.DR[next_sensor_index],2)
                        else:
                            #0 means the data is transmitted in the current position
                            #This check is needed in order to ensure that the data is not transmitted if the datarate is to low
                            if self.DR[visitied_sensor[-1]]>20:
                                temporary_AoI[data]= time + round(self.DS[data]/self.DR[visitied_sensor[-1]],2)
                            else:
                                temporary_AoI[data]=time+1000
    
                    list_sum_temporary_AoI.append(sum(temporary_AoI))
                    
                #extract the element with the lower AoI    
                min_AoI=100000*self.N
                for index, aoi in enumerate(list_sum_temporary_AoI):
                    if aoi<min_AoI:
                        min_AoI=aoi
                        index_max=index
                
                index_max_bin=self.integer_to_binary_list(index_max,len(data_in_memory_index))
                for j, el in enumerate(index_max_bin):
                    if el==0:
                        #If there is 0 the data isn't transmitted, so I just set the TX_data variable to 0
                        TX_data[data_in_memory_index[j]]=0
                    else:
                        #If there is 1 the data is transmitted from the current location. All the tracking lists have to be updated
                        time=time+round(self.DS[data_in_memory_index[j]]/self.DR[visitied_sensor[-1]],2)
                        transmission_position[data_in_memory_index[j]]=visitied_sensor[-1]+1
                        AoI[data_in_memory_index[j]]=time
                        TX_data[data_in_memory_index[j]]=1
                        data_in_memory[data_in_memory_index[j]]=0
                time_list.append(time) 

            #update the time
            time+=travel_time 
            time_list.append(time)
            for i in range(len(AoI)):
                if TX_data[i]==0:
                    #update the the AoI with the arrival time in the next city
                    AoI[i]=time
            #If all the sensors have been visited, the drone can come back 
            if len(visitied_sensor)>=self.N:
                done=True
            
            visitied_sensor.append(next_sensor_index)
            current_location=self.coordinates[next_sensor_index+1]
        
        #Once all the sensors have been visited, the drone travel back to the base station          
        time=time+(self.distance(current_location, (0,0))/self.v)
        time_list.append(time)  
        for i in range(len(AoI)):
                if TX_data[i]==0:
                    #Update the the AoI fot the data that have not been trasmitted. The AoI of these data corresponds to the time of arrival at the station base
                    AoI[i]=time 
        
        #The lists generated by the heuristic algorithm have to be manipulated in order to fit with the plot function
        visitied_cities_rescaled, list_tx_reorder, time_spent=self._data_manipulation( transmission_position, visitied_sensor, time_list )
        

        return AoI, visitied_cities_rescaled, list_tx_reorder, time_spent
    
    
    def transmission_algorithm(self, b_real, b, visited_sensor_rescaled, DS):
    
        done=False
        AoI=[0 for _ in range(self.N)]
        data_memory=[0 for _ in range(self.N)]
        transmission_position=[0 for _ in range(self.N)]
        time_visit_list=[0]
        TX_data=[0 for _ in range(self.N)]
        
        actual_sensor_index=1
        next_sensor=visited_sensor_rescaled[actual_sensor_index]
        
        time=round(self.distance(self.sensor_positions[0],self.sensor_positions[next_sensor+1])/self.speed)
        time_visit_list.append(time)

        while not(done):
            
            current_sensor=next_sensor
            next_sensor=visited_sensor_rescaled[actual_sensor_index+1]
            travel_time=round(self.distance(self.sensor_positions[current_sensor+1],self.sensor_positions[next_sensor+1])/self.speed)
            
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
            
            
            min_AoI=100000*self.N
            for index, aoi in enumerate(sum_AoI):
                #print(sum(aoi))
                if aoi<min_AoI:
                    min_AoI=aoi
                    index_min=index

            index_min_bin=self.integer_to_binary_list(index_min, len(index_data))
            
            for j, num in enumerate(index_min_bin):
                #Even if the drone need to transmitt, it need to check if the connection is present 
                if num==0 :
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
        travel_time=round(self.distance(self.sensor_positions[current_sensor+1], (0,0))/self.speed)
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
        
        
        min_AoI=100000*self.N
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

        _ , list_tx_reorder, time_spent=self._data_manipulation_2( transmission_position, visited_sensor_rescaled, time_visit_list )
        
        return AoI, list_tx_reorder, time_spent, transmission_position



    def path_planning_heuristics(self, DR, coordinates):
        current_location=(0,0)
        visited_sensors=[-1] 
        print(DR)
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
                    if DR[i]==1:
                        W.append(round(D[i]/(1),4))
                    else:
                        W.append(round(D[i]/(math.log10(DR[i])),4))


            #The next sensor is the one with the lowest weight
            next_sensor_index = W.index(min(W))
            visited_sensors.append(next_sensor_index)
            current_location=coordinates[next_sensor_index+1]

        return visited_sensors



    def optimizer_compute_2(self):
        _visited_sensor_rescaled=self.path_planning_heuristics(self.DR, self.coordinates)
        AoI, list_tx_reorder, time_spent, transmission_position=self.transmission_algorithm(self.DR, self.DR, _visited_sensor_rescaled, self.DS)
        visited_sensor=[position+1 for position in _visited_sensor_rescaled]
        #draw_line_between_points(1,visited_sensor, sensor_positions[:M],list_tx_reorder, time_spent,  upper_bound, lower_bound, "Reference Solution", 1, sum(AoI), actual_DR)
        return AoI, visited_sensor, list_tx_reorder, time_spent  










    def _data_manipulation_2(self, transmission_position, visitied_sensor, time_list ):
        tx_base_station=[]
        list_tx_reorder=[]
        print(time_list)
        #change the structure of the transmission_position list
        for i in range(self.N+1):
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

