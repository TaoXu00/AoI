import random
import numpy as np
class Generate_synthetic_dataset:
    def __init__(self, config):
        self.min_x = int(config['Gen_Synthetic_Dataset']['min_x'])
        self.max_x = int(config['Gen_Synthetic_Dataset']['max_x'])
        self.min_y = int(config['Gen_Synthetic_Dataset']['min_y'])
        self.max_y = int(config['Gen_Synthetic_Dataset']['max_y'])
        self.min_bitrate = int(config['Gen_Synthetic_Dataset']['min_bitrate'])
        self.max_bitrate = int(config['Gen_Synthetic_Dataset']['max_bitrate'])
        self.dir= config['Gen_Synthetic_Dataset']['dir_synthetic_dataset']
        self.sample_size= int(config['Gen_Synthetic_Dataset']['sample_size'])
        self.matrix_width= int(config['Gen_Synthetic_Dataset']['matrix_width'])
        self.matrix_height = int(config['Gen_Synthetic_Dataset']['matrix_height'])
        self.max_num_sensor = int(config['Gen_Synthetic_Dataset']['max_num_sensors'])

    def generate_random_bitrate_matrix(self, noises):  # this bitrate matrix is a 10x10 matrixs.
        bitrate_2D = np.array([[ random.uniform(self.min_bitrate, self.max_bitrate) for _ in range(10)] for _ in range(10)])
        bitrate_2D[bitrate_2D<100]=1
        np.savetxt(f'{self.dir}synthetic_bitrate_mean.txt', bitrate_2D, delimiter=',')
        # generate the std file with different level of noise from 10%-50%
        for noise in noises:
            std = [[bitrate_2D[i][j] * noise if bitrate_2D[i][j] !=1 else 0 for j in range(10)] for i in range(10)]
            np.savetxt(f'{self.dir}synthetic_bitrate_std_{noise}.txt', std, delimiter=',')

    def read_coordinates_from_file_with_bracket(self,filename):
        coordinates = []
        with open(filename, 'r') as file:
            for line in file:
                prov = line.split('(')[1]
                line = prov.split(')')[0]
                x, y = map(int, line.strip().split(','))
                coordinates.append((x, y))
        return coordinates

    def read_bitrate_from_file(self, file_path: str, sensor_positions: list):
        delimiter = ","
        # Read the matrix from the text file
        with open(file_path, "r") as file:
            # Read lines into a list
            lines = file.readlines()

        # Convert each line to a list of numbers (float by default)
        bitrate_matrix = np.array([list(map(float, line.strip().split(delimiter))) for line in lines])

        bitrate_coordinate = []
        for coor in sensor_positions:
            cell_index = self.is_within_matrix(coor[0], coor[1], self.min_x, self.max_x, self.min_y, self.max_y,
                                               self.matrix_width, self.matrix_height)

            # if bitrate_matrix[cell_index] < 123:  # in case there is no connection
            #     bitrate_coordinate.append(1)
            # else:
            bitrate_coordinate.append(int(bitrate_matrix[cell_index]))

        bitrate_coordinate[0] = 10000

        return bitrate_coordinate

    def read_std_from_file(self, file_path: str, sensor_positions: list):
        delimiter = ","
        # Read the matrix from the text file
        with open(file_path, "r") as file:
            # Read lines into a list
            lines = file.readlines()

        # Convert each line to a list of numbers (float by default)
        bitrate_matrix = np.array([list(map(float, line.strip().split(delimiter))) for line in lines])

        bitrate_coordinate = []
        for coor in sensor_positions:
            cell_index = self.is_within_matrix(coor[0], coor[1], self.min_x, self.max_x, self.min_y, self.max_y,
                                               self.matrix_width, self.matrix_height)

            bitrate_coordinate.append(int(bitrate_matrix[cell_index]))

        bitrate_coordinate[0] = 10000

        return bitrate_coordinate

    def random_data_generator(self, num_sensor, lower_bound, upper_bound):

        data_random_list = []
        for _ in range(num_sensor):
            data_random = random.uniform(lower_bound, upper_bound)
            data_random_list.append(int(data_random))

        return data_random_list

    def sensor_map_and_bite_rate_generation(self,sensor_map_direct, sensor_map_name, biterate_mean_file_path,
                                            bitrate_std_file_path, num_sensor):
        # import coordinate from the file
        sensor_positions = self.read_coordinates_from_file_with_bracket(
            sensor_map_direct + sensor_map_name)  # the sensor positions includes the base station (the first element) with (0,0)

        bit_rate_list = self.read_bitrate_from_file(biterate_mean_file_path, sensor_positions[:])
        standard_dev = self.read_std_from_file(bitrate_std_file_path, sensor_positions[:])
        # generate random data size

        data_size = self.random_data_generator(num_sensor,2000, 2000)  # it seems we use different data size for each sensor
        return sensor_positions, bit_rate_list, standard_dev,data_size

    def generate_dataset(self, bit_rate_list, standard_dev, sample_size, noise, num_sensor):
        """
        Generates a dataset where each row is a realization vector including every bit rate at the same time instance.

        :param bit_rate_list: List of bit rates (mean values for normal distributions).
        :param standard_dev: Standard deviation for the normal distributions.
        :param sample_size: Number of realization vectors to generate (i.e., number of rows).
        :return: A 2D array where each row is a realization vector, and each column corresponds to a bit rate.
        """
        dataset=[]
        # Fill the dataset with samples for each bit rate
        for i in range(len(bit_rate_list)):
            # Generate samples for this bit rate and fill the corresponding column
            data= np.random.normal(bit_rate_list[i],standard_dev[i], size=sample_size)
            dataset.append(data)
        dataset=np.array(dataset).T
        for i in range(sample_size):
            for j in range(len(bit_rate_list)):
                if dataset[i][j] < 10:
                    dataset[i][j]=1
        np.savetxt(self.dir+f'dataset_{sample_size}_{num_sensor}_{noise}.txt', dataset, delimiter=',', fmt='%.5f')
        return dataset
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


    def generate_synthetic_dataset(self):
        # Generate random coordinates
        num_sensor_list = [20, 40, 60, 80, 100]
        noises = [0.15, 0.2, 0.25, 0.3]
        coordinates = [(int(random.uniform(self.min_x, self.max_x)), int(random.uniform(self.min_y, self.max_y))) for _
                       in range(self.max_num_sensor)]
        for num_sensor in num_sensor_list:
            # Write coordinates to the file
            with open( f'{self.dir}sensor_{num_sensor}_map','w') as file:
                file.write(f"(0, 0)\n")
                for coord in coordinates[:num_sensor]:
                    file.write(f"({coord[0]}, {coord[1]})\n")
        # generate the random bitrate matrix
        self.generate_random_bitrate_matrix(noises)
        #generate the synthetic data rate for each map with each noise level
        #first generate the data rate for the map with 100 sensors
        dir_sensor_map = self.dir
        for noise in noises:
            sensor_map_name = 'sensor_' + str(self.max_num_sensor) + '_map'
            biterate_mean_file_path = self.dir + 'synthetic_bitrate_mean.txt'
            bitrate_std_file_path = self.dir + 'synthetic_bitrate_std_' + str(noise) + '.txt'
            sensor_positions, bit_rate_list, standard_dev, data_size = self.sensor_map_and_bite_rate_generation(
                dir_sensor_map, sensor_map_name, biterate_mean_file_path, bitrate_std_file_path, num_sensor)
            dataset= self.generate_dataset(bit_rate_list[1:num_sensor + 1], standard_dev[1:num_sensor + 1], self.sample_size,
                                  noise, num_sensor)

            for num_sensor in num_sensor_list:
                sensor_dataset= dataset[:, :num_sensor]
                np.savetxt(self.dir + f'dataset_{self.sample_size}_{num_sensor}_{noise}.txt', sensor_dataset, delimiter=',',
                           fmt='%.5f')
        # for num_sensor in num_sensor_list:
        #     for noise in noises:
        #         dir_sensor_map=self.dir
        #         sensor_map_name='sensor_'+str(num_sensor)+'_map'
        #         biterate_mean_file_path=self.dir+'synthetic_bitrate_mean.txt'
        #         bitrate_std_file_path=self.dir+'synthetic_bitrate_std_'+str(noise)+'.txt'
        #         sensor_positions, bit_rate_list, standard_dev, data_size = self.sensor_map_and_bite_rate_generation(dir_sensor_map, sensor_map_name, biterate_mean_file_path, bitrate_std_file_path, num_sensor)
        #         self.generate_dataset(bit_rate_list[1:num_sensor+1], standard_dev[1:num_sensor+1], self.sample_size, noise, num_sensor)
