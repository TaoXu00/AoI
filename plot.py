import matplotlib.pyplot as plt
class plotter:
    def __init__(self, directory):
        self.directory = directory

    def plot_trajectory_distances(self,distances, reference_length):
        """
        Plots the trajectory distances and adds a reference trajectory length as a horizontal line.

        :param distances: List of trajectory distances.
        :param reference_length: Reference trajectory length to be plotted as a horizontal line.
        """
        # Create a figure and axis
        plt.figure()
        # Plot the trajectory distances
        plt.plot(distances, label='Trajectory Distances')

        # Add a reference horizontal line
        plt.axhline(y=reference_length, color='r', linestyle='--', label='Reference Length with perfect knowledge ')

        # Add labels and title
        plt.xlabel('Iterations')
        plt.ylabel('Trajectory Length')
        plt.title('Trajectory Distances with Reference Length')
        # Add legend
        plt.legend()
        # Show grid
        plt.grid(True)
        plt.savefig(self.directory+'trajectory_length_over_iterations')
        # Display the plot
        #plt.show()

    def plot_instant_AoI(self, AoI_estimated):
        plt.figure()
        instant_regret = [AoI_estimated[i] for i in range(len(AoI_estimated))]
        plt.plot(instant_regret)  # marker='o' adds points on the line
        plt.xlabel('Trials')
        plt.ylabel('Loss')
        plt.title('Instant AOI: ')
        plt.grid(True)
        plt.savefig(self.directory+'instant_AoI.png')

    def plot_mean_error_b_hat(self,ELE,run,b_hat_list, b_true):
        #compute the error
        N=0
        for i in range(ELE):
            if b_true[i]==1:
                continue
            else:
                N+=1

        error_list=[]
        for i in range(ELE):
            error=[]
            error=[b_hat_list[j][i]-b_true[i] for j in range(run-1)]
            error_list.append(error)

        #print(error_list)
        #compute the mean value error vector
        mean_error=[]
        for i in range(run-1):
            sum_error=0
            for j in range(ELE):
                if b_true[j]==1:
                    continue
                sum_error += abs(error_list[j][i])

            mean_error.append(sum_error/N)

        plt.figure()
        plt.plot(mean_error, color='blue')  # marker='o' adds points on the line
        plt.xlabel('Trials')
        plt.ylabel('Error')
        plt.title('b_hat mean error')
        plt.grid(True)
        plt.savefig(self.directory+'overall_bitrate_learning_error.png')


    def plot_average_cumulative_AoI(self, run, AoI_estimated, AoI_sum_ref):
        expeted_value_AoI=[AoI_sum_ref for _ in range(run)]
        cumulative_aoi = []
        cummulative_aoi_with_windown=[]
        cumulative_sum = 0
        window_size = 100
        for i in range(len(AoI_estimated)):
            cumulative_sum = sum(AoI_estimated[:i+1])/(i+1)
            #cumulative_sum = sum(AoI_estimated[i:i+window_size]) / window_size
            cumulative_aoi.append(cumulative_sum)

        for i in range(len(AoI_estimated)-window_size+1):
            #cumulative_sum = sum(AoI_estimated[:i+1])/(i+1)
            cumulative_sum = sum(AoI_estimated[i:i+window_size]) / window_size
            cummulative_aoi_with_windown.append(cumulative_sum)

        #reverse moving average
        # Reverse the data
        reversed_data = AoI_estimated[::-1]
        moving_averages_reversed = []
        for i in range(len(reversed_data) - window_size + 1):
            window = reversed_data[i:i + window_size]
            window_average = sum(window) / window_size
            moving_averages_reversed.append(window_average)
        # Reverse the moving averages to match the original data order
        cumulative_aoi_with_window_reversed= moving_averages_reversed[::-1]

        #used to save the cumulative AOI in a file
        with open(f"{self.directory}cumulative_aoi.txt", "w") as file:
             file.write(", ".join(map(str, AoI_estimated)))
             file.write("\n")
             file.write(", ".join(map(str, cummulative_aoi_with_windown)))
             file.write("\n")
             file.write(", ".join(map(str, cumulative_aoi_with_window_reversed)))
             file.write("\n")

        plt.figure()
        plt.plot(cumulative_aoi, color='blue')  # marker='o' adds points on the line
        plt.plot(expeted_value_AoI, color='red')
        plt.xlabel('Trials')
        plt.ylabel('AoI')
        plt.title('Average of cumulative AoI')
        plt.grid(True)
        plt.savefig(self.directory + 'avg_commulative_AoI.png')

        plt.figure()
        plt.plot(cummulative_aoi_with_windown, color='blue')  # marker='o' adds points on the line
        plt.plot(expeted_value_AoI, color='red')
        plt.xlabel('Trials')
        plt.ylabel('AoI')
        plt.title('Average of cumulative AoI with moving window')
        plt.grid(True)
        plt.savefig(self.directory + 'avg_commulative_AoI_with_moving_window.png')

        plt.figure()
        plt.plot(cumulative_aoi_with_window_reversed, color='blue')  # marker='o' adds points on the line
        plt.plot(expeted_value_AoI, color='red')
        plt.xlabel('Trials')
        plt.ylabel('AoI')
        plt.title('Average of cumulative AoI with moving window_reversed')
        plt.grid(True)
        plt.savefig(self.directory + 'avg_commulative_AoI_with_moving_window_reversed.png')


    def plot_b_hat(self, run, b_hat, bit_rate_list, b_hat_list, b_real_list) :
        for i in range(len(b_hat)):
            expeted_value=[bit_rate_list[i+1] for _ in range(run)]
            plot_func=[b_hat_list[j][i] for j in range(run-1)]
            plot_b_real=[b_real_list[j][i] for j in range((run-1))]
            plt.figure()
            plt.plot(plot_func, color='blue', label='b_hat')
            plt.plot(expeted_value, color='red', label='Mean Value')
            plt.plot(plot_b_real, 'go', label='Real Value' )
            plt.legend()
            plt.xlabel('Iteration')
            plt.ylabel('Data Rate')
            plt.title('B_hat sensor '+str(i+1))
            plt.grid(True)

    def plot_regret(self, AoI_estimated, AoI_real, c):
        #regret_AoI=[(i+1)*AoI_real - sum(AoI_estimated[:i])  for i in range(len(AoI_estimated))]
        instant_regret=[AoI_real-AoI_estimated[i]for i in range(len(AoI_estimated))]
        cumulative_regret = []
        cumulative_sum = 0
        for i,_ in enumerate(instant_regret):
            cumulative_sum = sum(instant_regret[:i+1])/(i+1)
            cumulative_regret.append(cumulative_sum)

        plt.figure()
        plt.plot(cumulative_regret)  # marker='o' adds points on the line
        plt.xlabel('Trials')
        plt.ylabel('Regret')
        plt.title('Average of Cumulative regret with c: ' + str(c) )
        plt.grid(True)
        plt.savefig(self.directory+'regret.png')


    def plot_b_mod(self, run, b_hat, bit_rate_list, b_mod_list, b_hat_list):
        for i in range(len(b_hat)):
            plot_func=[b_mod_list[j][i] for j in range(run-1)]
            plot_func2=[b_hat_list[j][i] for j in range(run-1)]
            plt.figure()
            plt.plot(plot_func, color='blue', label='b_mod')
            plt.plot(plot_func2, color='red', label='b_hat')
            plt.legend()
            plt.xlabel('Iteration')
            plt.ylabel('Data Rate')
            plt.title('B_mod sensor '+str(i+1))
            plt.grid(True)


    def plot_moving_avarage_AoI(self, run, AoI_estimated, AoI_real, data_window:list):

        expeted_value_AoI=[AoI_real for _ in range(run)]

        plt.figure()
        for window in data_window:
            m_avarage=self.moving_average(AoI_estimated, window)
            plt.plot(m_avarage, label= str(window))  # marker='o' adds points on the line

        plt.plot(expeted_value_AoI, color='red')
        plt.legend()
        plt.xlabel('Trials')
        plt.ylabel('AoI')
        plt.title('Moving Avarage AoI')
        plt.grid(True)


    def moving_average(self, data, window_size):
        """
        Calculate the moving average of a list with a given window size.

        :param data: List of numerical values.
        :param window_size: The size of the moving window.
        :return: List of moving averages.
        """
        if window_size <= 0:
            raise ValueError("Window size must be greater than 0")

        if len(data) < window_size:
            raise ValueError("Window size must be less than or equal to the length of the data list")

        moving_averages = []
        for i in range(len(data) - window_size + 1):
            window = data[i:i + window_size]
            window_average = sum(window) / window_size
            moving_averages.append(window_average)

        return moving_averages
