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

# Create folders
def create_folder(path): 
    if not os.path.exists(path):
        os.makedirs(path)

# Define path to load files
piezo_data_path =    os.path.join(script_dir, "data", "recorded_data", test, folder, f'piezo_{finger}.npy')
sensor_values_path = os.path.join(script_dir, "data", "recorded_data", test, folder, 'sensor_data.npy')
trigger_data_path =  os.path.join(script_dir, "data", "recorded_data", test, folder, 'received_trigger.npy')

# Define path to save files
create_folder(os.path.join(script_dir, "data_txt"))

piezo_values = np.load(piezo_data_path)  # Finger sensor data
sensor_values = np.load(sensor_values_path)  # Force sensor data
trigger_values = np.load(trigger_data_path)  # Trigger data

fs_sensor = 500  # Hz
fs_piezo = 313  # Hz (approx)

duration = trigger_values[-1][-1]
fs_trigger =  len(trigger_values[-1])/ duration

print("Duration force snesor = ", len(sensor_values)/fs_sensor)
print("Duration piezo sensor = ", len(piezo_values)/fs_piezo)

start_time = 0  # seconds
piezo_values = piezo_values[int(start_time * fs_piezo):int(duration * fs_piezo)]
sensor_values = sensor_values[int(start_time * fs_sensor):int(duration * fs_sensor)]
print("New sensor duration = ", len(sensor_values)/fs_sensor)
print("New piezo duration = ", len(piezo_values)/fs_piezo)

# Save txt
np.savetxt(f"data_txt\piezo_data.txt", piezo_values)
np.savetxt(f"data_txt\force_data.txt", sensor_values)