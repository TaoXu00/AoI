import UCB_version_3 as UCB

class exp1:

    def run_exp1(self, config):
        num_of_sensors_list=[20]
        noise=config['Exp1']['noise']
        for num_sensors in num_of_sensors_list:
            config['UCB3']['dir_ucb3']= f'experiment_results/synthetic_dataset_UCB3/{num_sensors}sensors/'
            config['UCB3']['dir_bit_rate_std'] = f'synthetic_dataset/synthetic_bitrate_std_{noise}.txt'
            config['UCB3']['dir_dataset']= f'synthetic_dataset/dataset_501_{num_sensors}_{noise}.txt'
            config['UCB3']['sensor_map_name']=f'sensor_{num_sensors}_map'
            config['UCB3']['num_sensor']= str(num_sensors+1)
            config['UCB3']['n_trials_path']=config['Exp1']['n_trials_path']
            config['UCB3']['n_trials_transmission']=config['Exp1']['n_trials_transmission']
            ucb3 = UCB.UCB3(config)
            ucb3.run_UCB()
