import numpy as np
import pandas as pd
import seaborn as sns
from hmmlearn import _hmmc
import matplotlib.pyplot as plt
import os

if os.name == 'nt':
    os.system('cls')

print("PREDICION OF THE WEATHER")
# Define hidden states
para = ['Sunny', 'Rain']
n_para = len(para)
print("no. of hidden states = ", n_para)

# Define observations
obs = ["Dry", "Wet"]
n_obs = len(obs)
print("no. of observations = ", n_obs)

# Initial states distribution
para_prob = np.array([0.6, 0.4])
print(f"Para prob = \n", para_prob)

# Transition probabilities
trans_prob = np.array([[0.7, 0.3], 
                      [0.3, 0.7]])
print(f"Trans prob = \n", trans_prob)

# Emission probabilities
emis_prob = np.array([[0.9, 0.1], 
                      [0.2, 0.8]])
print(f"Emis prob = \n", emis_prob)
