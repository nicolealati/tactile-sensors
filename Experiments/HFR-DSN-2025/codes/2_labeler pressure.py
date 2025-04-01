import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import os

os.system('cls')

finger="index"
test_n = 1
test = "pressure"
folder=f"{test}_{finger}_{test_n}"

# Define current folder
script_dir = os.path.dirname(os.path.abspath(__file__))

labels_levels=[]
labels_on_off=[]

# Create folders
def create_folder(path): 
    if not os.path.exists(path):
        os.makedirs(path)

# Define the relative path to the file
create_folder(os.path.join(script_dir, "data", "labeled_data", folder))
sensor_values_path = os.path.join(script_dir, "data", "downsampled_data",  folder, 'sensor_data_downsampled.npy')
labels_path =        os.path.join(script_dir, "data", "labeled_data", folder, 'labels_levels.npy')
labels_path_levels = os.path.join(script_dir, "data", "labeled_data", folder, 'labels_on_off.npy')

# Load the data
sensor_values = abs(np.load(sensor_values_path)) 
plt.plot(sensor_values[:,2])
#plt.show()

for i in range(len(sensor_values[:,2])):

    # Pressure / No pressure
    if sensor_values[i][2] < 0.2:
        labels_on_off.append(0)
    else:
        labels_on_off.append(1)

    # Pressure (levels))
    if sensor_values[i][2] < 1:
        labels_levels.append(0)
    elif sensor_values[i][2] < 3:
        labels_levels.append(1)
    elif sensor_values[i][2] < 5:
        labels_levels.append(2)
    elif sensor_values[i][2] < 7:
        labels_levels.append(3)
    else:
        labels_levels.append(4)
labels_levels = np.array(labels_levels)

print()
print("Length sensor values = ", len(sensor_values))
print("Length labels = ", len(labels_levels))

plt.plot(labels_levels)
#plt.show()

np.save(labels_path, labels_levels)
np.save(labels_path_levels, labels_on_off)

print()

