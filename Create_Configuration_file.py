import configparser

config =configparser.ConfigParser()
config['Dir'] = {
            'dir_bit_rate_mean':  'Bitrate_map/my_matrix_bitrate_mean.txt',
            'dir_bit_rate_std' : 'Bitrate_map/my_matrix_bitrate_std.txt',
    }
config['Run'] = {
    'experiments': 'UCB3'
}
config['Drone'] = {
            'speed' : 5  #m/s
    }
config['UCB3'] = {
            'dir_UCB3': 'experiment_results/UCB3/',
            'num_sensor':  25,
            'N_trials_path' : 50,
            'N_trials_transmission' : 10,
            'c' : 15,
            'matrix_width' : 10,
            'matrix_height' : 10,
            'map_upper_bound': (500, 500),
            'map_lower_bound': (-500, -500),
            'min_x' : -500,
            'max_x' : 500,
            'min_y': 500,
            'max_y': 500
    }

with open('config.ini', 'w') as config_file:
            config.write(config_file)