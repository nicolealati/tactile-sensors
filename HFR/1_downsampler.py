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
trigger_data_path =  os.path.join(script_dir, "data", "recorded_data", test, folder, 'received_trigger.npy')
sensor_values_path = os.path.join(script_dir, "data", "recorded_data", test, folder, 'sensor_data.npy')

# Define path to save files
create_folder(os.path.join(script_dir, "data", "downsampled_data", folder))
saved_path_sensor =  os.path.join(script_dir, "data", "downsampled_data", folder, 'sensor_data_downsampled.npy')
saved_path_trigger = os.path.join(script_dir, "data", "downsampled_data", folder, f'triggers_downsampled.npy')
saved_path_piezo =   os.path.join(script_dir, "data", "downsampled_data", folder, f'piezo_{finger}_downsampled.npy')

# Load the data
piezo_values = np.load(piezo_data_path)  # Finger sensor data
sensor_values = np.load(sensor_values_path)  # Force sensor data
trigger_values = np.load(trigger_data_path)  # Trigger data
 
# Define duration
duration = trigger_values[-1][-1]
print("Duration = ", duration)

# Calculate trigger frequency
fs_trigger =  len(trigger_values[-1])/ duration
print("Trigger freq = ", fs_trigger)

# Original sample rates
fs_sensor = 500  # Hz
fs_piezo = 313  # Hz (approx)

# Define seconds to remove
start_time = 7  # seconds
print()
print("Duration force snesor = ", len(sensor_values)/fs_sensor)
print("Duration piezo sensor = ", len(piezo_values)/fs_piezo)
trigger_values = trigger_values[:, int(start_time * fs_trigger):]
sensor_values = sensor_values[int(start_time * fs_sensor):int(duration * fs_sensor)]
piezo_values = piezo_values[int(start_time * fs_piezo):int(duration * fs_piezo)]
print("New trigger duration = ", len(trigger_values[0])/fs_trigger)
print("New sensor duration = ", len(sensor_values)/fs_sensor)
print("New piezo duration = ", len(piezo_values)/fs_piezo)

# Calculate the time vectors
time_sensor = np.linspace(0, duration, len(sensor_values))
time_piezo = np.linspace(0, duration, len(piezo_values))
time_trigger = np.linspace(0, duration, len(trigger_values[0]))
 
# Interpolate the force sensor data
interp_func_sensor = interp1d(time_sensor, sensor_values, axis=0, kind='linear')
sensor_values_resampled = interp_func_sensor(time_piezo)

trigger_values_resampled = np.zeros((trigger_values.shape[0], len(piezo_values)))
for i in range(trigger_values.shape[0]):
    interp_func_trigger = interp1d(time_trigger, trigger_values[i], kind='linear')
    trigger_values_resampled[i] = interp_func_trigger(time_piezo)
 
# Check the shapes to confirm matching sample numbers
print(f"\nShapes:")
print(trigger_values_resampled.T.shape, sensor_values_resampled.shape, piezo_values.shape)

# Save the resampled data
np.save(saved_path_sensor, sensor_values_resampled)
np.save(saved_path_trigger, trigger_values_resampled)
np.save(saved_path_piezo, piezo_values)

print()