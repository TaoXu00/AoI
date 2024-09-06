#Path finding for with the bitrate from the real world from the real world
import numpy as np
import random
import matplotlib.pyplot as plt
import matplotlib
from gurobipy import *

class gurobiOptimezerSol:
    def __init__(self, N, speed, cooridinate, datarate, datasize):  # Constructor method
        self.N = N
        self.speed=speed #[m/s]
        self.sensor_positions=cooridinate
        self.bitrate_coordinate=datarate
        self.data_size=datasize


    def distance(self,sensor1, sensor2):
        x1, y1 = sensor1
        x2, y2 = sensor2
        return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

    def dx_dy(self, sensor1, sensor2):
        x1, y1 = sensor1
        x2, y2 = sensor2
        return x2 - x1, y2 - y1

    
    def plot_sensor_positions(self, sensor_positions, upper_bound, lower_bound):
        """
        Plots the sensor positions on a matplotlib scatter plot.

        Args:
            sensor_positions: A list of tuples representing sensor positions (x, y).
            limit_coord: A tuple representing the limit coordinates (x_max, y_max).
        """
        
        y, x = zip(*sensor_positions)  # Unpack sensor positions into separate lists
        
        plt.figure(figsize=(8, 6))  # Set figure size
        plt.scatter(x[0], y[0], marker='*', color='green', alpha=1, label='Starting point')  # Plot sensors
        plt.scatter(x[1:-1], y[1:-1], marker='o', color='blue', alpha=0.8, label='Sensors')  # Plot sensors
        
        values = np.linspace(lower_bound, upper_bound, 11) 
        #print the row of the grid
        for el in values:  
            plt.plot( [lower_bound, upper_bound], [el,el], 'c--', linewidth=0.5)
            plt.plot([el,el], [lower_bound, upper_bound],  'c--', linewidth=0.5 )
        

        plt.xlim(lower_bound[0], upper_bound[0])  # Set x-axis limits
        plt.ylim(lower_bound[1], upper_bound[1])  # Set y-axis limits
        plt.xlabel('X-Coordinate')
        plt.ylabel('Y-Coordinate')
        plt.title('Sensor Positions')
        plt.grid(False)
        plt.legend()
        plt.show()

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
        
        print(time_spent)
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
            plt.text(510, 445-(i*35),"S"+str(i+1)+": "+str(bitrate[i+1]), color='red')
            
            #plt.text(-530+(i*120),600,"S"+str(i+1)+"="+str(bitrate[i+1]), color='red')
            if transmission_position[i]:
             plt.text(ele[0]+10, ele[1]+60,str(transmission_position[i]),color='green')
            else:
             plt.text(ele[0]+10, ele[1]+60, "NaN",color='green') 
        
        
        """  #position of transmission plot
        for i,ele in enumerate(list_visited_sensor_ordered[1:]):
            
            if transmission_position[ele-1]:
                plt.text(-530+(i*100),550, "S"+str(ele)+": "+str(transmission_position[ele-1]))
            else:
                plt.text(-530+(i*100),550, "S"+str(ele)+": "+"NaN") """



        if len(transmission_position)==self.N:
            plt.text( 0+10,0+60,str(transmission_position[self.N-1]), color='green')
        else:
            plt.text(0+10, 0+60,"NaN", color='green')    
        
        description="   AoI: "+ str(AoI)   
        plt.figtext(0.5,-0.002, description, ha='center', va='bottom', fontsize=20)  # Adjust position and size as needed
        #plt.text(-400, -610, description, fontsize=20)
        #plt.xlabel('X-Coordinate')
        plt.ylabel('Y-Coordinate')
        plt.title(title)
        plt.xlim(lower_bound[0], upper_bound[0])  # Set x-axis limits
        plt.ylim(lower_bound[1], upper_bound[1])  # Set y-axis limits
        plt.grid(False)
        plt.legend()

        #insert the plt.show() after the call of the function
        #plt.show()


    def read_coordinates_from_file(filename):
            coordinates = []
            with open(filename, 'r') as file:
                for line in file:
                    x, y = map(int, line.strip().split(','))
                    coordinates.append((x, y))
            return coordinates
    
    def initialize_env(self):

        upper_bound = (500, 500)  # Limit coordinates (x_max, y_max)
        lower_bound=(-500,-500)

        self.starting_point=0

        #this rapresents the TA_MATRIX used to send the data form the drone to the base station. Now it is constant, later it will be a function and willvary for every sensors
        self.data_sending_time=0

        #add the battery constraints
        self.hovering_energy=2 #[mAh/s]
        self.travelling_energy=10 #[mAh/s]
        self.battery_limit=3300

        self.tx_time=[]
        tx_decision=[]
        for i in range(len(self.bitrate_coordinate)):
            if self.bitrate_coordinate[i]>1: 
                self.tx_time.append(round(self.data_size[i]/self.bitrate_coordinate[i]))
                tx_decision.append(10000)
            else:
                self.tx_time.append(0)
                tx_decision.append(0)   

        self.travel_time =[]
        # Define travel TA_MATRIX matrix
        for i in range(self.N):
            travel_time_temp=[]
            for j in range(self.N):
                travel_time_temp.append(round(self.distance(self.sensor_positions[i], self.sensor_positions[j])/self.speed))
            self.travel_time.append(travel_time_temp)


        #self.plot_sensor_positions(self.sensor_positions, upper_bound, lower_bound)
        #print(self.sensor_positions)


    def optimizer_complete(self):
        
        transmission_time =[]
        # Define travel TA_MATRIX matrix
        for i in range(self.N):
            transmission_time_temp=[]
            for j in range(self.N):
                transmission_time_temp.append(round(self.data_size[i]/self.bitrate_coordinate[j]))
            transmission_time.append(transmission_time_temp)

        print(transmission_time)

        # Define model
        m = Model("tsp_different_time_tx_base_tx")
        #m.setParam('TimeLimit', 600)
    
        # Decision variables
        x = {}
        for i in range(self.N):
            for j in range(self.N):
                
                    x[i, j] = m.addVar(vtype=GRB.BINARY, name=f"x_{i}_{j}")

        u={}
        #print(x)
        for i in range(self.N):
            u[i] = m.addVar( vtype=GRB.INTEGER, name=f"u_{i}")

        TD={}
        for i in range(self.N):
            TD[i] = m.addVar( vtype=GRB.INTEGER, name=f"TD_{i}")

        TA={}
        for i in range(self.N):
            TA[i] = m.addVar( vtype=GRB.INTEGER, name=f"TA_{i}")


        AoI={}
        #The AoI is compute for the 9 sensors(not for the starting point)
        for i in range(self.N):        
            AoI[i] = m.addVar(vtype=GRB.INTEGER, name=f"AoI_{i}")
            

        TX={}
        for i in range(self.N):
            for j in range(self.N):
                TX[i,j] = m.addVar( vtype=GRB.BINARY, name=f"TX_{i}_{j}")

        y={}
        for i in range(self.N):
            y[i] = m.addVar( vtype=GRB.INTEGER, name=f"y_{i}")

        z={}
        for i in range(self.N):
            z[i] = m.addVar( vtype=GRB.INTEGER, name=f"z_{i}")

        #battery = m.addVar( vtype=GRB.INTEGER, name=f"battery")

        # Objective function--->Minimize the AoI
        m.setObjective(sum(AoI[i] for i in range(1,self.N)), GRB.MINIMIZE)


        #the data of every sensor has to be transmittedfrom one location or bring back to the base
        for i in range(1,self.N):
            m.addConstr(quicksum(TX[i, j] for j in range(self.N)) == 1)

        #the first sensors hasn't any data
        #m.addConstr(BT[0]+quicksum(TX[0, j] for j in range(1,N)) == 0)


        #----------------------------------------AOI---------------------------------------------------
        tau_m=0
        for i in range(1,self.N):
            m.addConstr(AoI[i]==(z[i]+quicksum(TX[i,j]*quicksum(TX[k,j]*transmission_time[k][j] for k in range(1,i+1)) for j in range(self.N))-tau_m))  
            #m.addConstr(AoI[i]==(quicksum(TX[i,j]*TA[j]+TX[i,j]*quicksum(TX[k,j]*transmission_time[k][j] for k in range(1,i+1)) for j in range(self.N))-tau_m)) 
            #m.addConstr(AoI[i]==(quicksum(TX[i,j]*TA[i]+quicksum(TX[k,j]*transmission_time[k][j] for k in range(1,N) if k<i) for j in range(1,N))+(BT[i]*TA[0])-tau_m))


        M=self.N*100
        for i in range(1,self.N):
            #m.addConstr(w[i]>=TA[0]-(1-BT[i])*M) 
            for j in range(self.N):
                m.addConstr(z[i]>=TA[j]-(1-TX[i,j])*M)   

        for i in range(self.N):
            m.addConstr(z[i]>=0)  
            
        #battery computation
        #m.addConstr(battery==quicksum((TD[i] - TA[i])*travelling_energy*x[i,j] for j in range(N) for i in range(N) if i!=j) + quicksum((TD[j] - TA[i])*hovering_energy*x[i,j] for j in range(1,N) for i in range(N)), "battery") 

        for j in range(self.N): 
            m.addConstr(TA[j]==(y[j]+quicksum(self.travel_time[i][j]*x[i,j] for i in range(self.N))))
            
        M=self.N*1000
        for j in range(self.N):
            for i in range(self.N):
                m.addConstr(y[j]>=TD[i]-(1-x[i,j])*M)
                        
        for i in range(1,self.N):          
            m.addConstr(TD[i]== TA[i]+ quicksum(TX[j,i]*round(transmission_time[j][i]) for j in range(self.N)))
                
        m.addConstr(TD[0]== 0) 
                        
        for i in range(self.N):
            # Ensure each city is visited at least once in any TA_MATRIX slot
            m.addConstr(quicksum(x[i, j] for j in range(self.N) if j != i ) == 1)

        for j in range(self.N):
            # Ensure each city is visited at least once in any TA_MATRIX slot
            m.addConstr(quicksum(x[i, j] for i in range(self.N) if j != i ) == 1)


        # for i in range(N):
        #     for j in range(N):
        #         m.addConstr(TX[i,j]*(u[j]-u[i])>=0)

        for i in range(self.N):
            for j in range(self.N):
                #m.addConstr(TX[i,j]*(u[j]-u[i])>=0)
                m.addConstr((u[i]-u[j])<=(1-TX[i,j])*self.N)
                
        m.addConstr(u[0]==self.N+1)
        for i in range(self.N):
            m.addConstr(u[i]<=self.N+1)
            m.addConstr(u[i]>=1)


        for i in range(1,self.N):
            for j in range(self.N):
                #if i!=j: 
                    m.addConstr(u[i]-u[j]+1<=self.N*(1-x[i,j]))
                    
                
        
        #back to the starting point constrants
        m.addConstr(quicksum(x[i, 0] for i in range(1,self.N))  == quicksum(x[0, j] for j in range(1,self.N)))

        m.optimize()

        new_tx_time=[]
        # Print solution
        if m.status == GRB.OPTIMAL:
            #print(sum(AoI[i].x for i in range(N)))
            total_AoI3 = m.objVal
            battery3=3000
            AoI3=[]
            print(f"Avarage AoI :{total_AoI3}")
            for i in range(self.N):
                if i>0:
                    AoI3.append(AoI[i].x)
                for j in range(self.N):
                    
                    if TX[i, j].x > 0.99:
                            print(f"Sensor's data {i} send to the base station from the position {j}")
                            new_tx_time.append(round(self.data_size[i]/self.bitrate_coordinate[j]))
                    if i!=j:
                        if x[i, j].x > 0.99:
                                print(f"City {i} -> City {j}  TA_MATRIX departure from city{i}={TD[i].x} TA_MATRIX arrival to city {j}={TA[i].x}")
                            
        else:
            print("No feasible solution found") 

        #print(u)

        time_list=[]
        for i in range(self.N):
            time_list_prov=[]
            for j in range(self.N):
                time_list_prov.append(int(TX[i,j].x))
            time_list.append(time_list_prov)
        for i in range(self.N):
            print(time_list[i][:])  

        print("\n")
        list_visited_sensor_ordered3=[]
        time_departure_sensor3=[]
        time_arrival_sensor3=[]

        #set the initial value
        list_visited_sensor_ordered3.append(0)
        time_departure_sensor3.append(0) 
        time_arrival_sensor3.append(TA[0].x)
        for index in range(self.N+1):
            for i in range(self.N):
                    if u[i].x==index:
                        list_visited_sensor_ordered3.append(i)
                        #time_visited_sensor2.append(TD[i].x)
                        time_departure_sensor3.append(TD[i].x) 
                        time_arrival_sensor3.append(TA[i].x)



        time_spent3=[str(time_arrival_sensor3[i])+"-->"+str(time_departure_sensor3[i]) for i in range(self.N) ]

        
        #print(list_visited_sensor_ordered3)

        base_tx=[]
        list_postion_send3_prov=[]
        for j in range(0,self.N):
            postion_send=[]
            # if BT[j].x==1:
            #   base_tx.append(j)
            for i in range(1,self.N):
                if TX[i,j].x==1: 
                    postion_send.append(i)
            list_postion_send3_prov.append(postion_send)

        list_postion_send3=list_postion_send3_prov[1:self.N]
        list_postion_send3.append(list_postion_send3_prov[0])
        print(AoI3)
        
        


        sol3='LoRa + Tx Decision with BS'
        

        return  AoI3, total_AoI3, list_postion_send3, time_spent3, list_visited_sensor_ordered3