import numpy as np
import Generate_synthetic_dataset
import UCB_version_3 as UCB
import configparser
import random
import exp1_varying_num_of_sensors as exp1


config = configparser.ConfigParser()
config.read('/Users/xutao/PycharmProjects/AoI-percom/config.ini')
#config.read('config.ini')
exp=config['Run']['experiment']
Generate_synthetic_dataset=Generate_synthetic_dataset.Generate_synthetic_dataset(config)


if  exp=='Gen_Synthetic_Dataset':
    Generate_synthetic_dataset.generate_synthetic_dataset()
elif exp == 'UCB3':
    ucb3= UCB.UCB3(config)
    ucb3.run_UCB()
elif exp=='Exp1':
    exp1 = exp1.exp1()
    exp1.run_exp1(config)






