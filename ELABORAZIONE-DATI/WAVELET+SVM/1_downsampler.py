import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import os

# Creazione di un semplice grafico
script_dir = os.path.dirname(os.path.abspath(__file__))

finger="index"
test_n = 1
folder=f"rolling_{test_n}_{finger}"
test = "rolling"

# Define the relative path to the file
piezo_data_path = os.path.join(script_dir, test, folder, f'{finger}.npy')
trigger_data_path = os.path.join(script_dir, test, folder, 'triggers.npy')
sensor_values_path = os.path.join(script_dir, test, folder, 'sensor_values.npy')

saved_path_sensor = os.path.join(script_dir, test, folder, 'sensor_values_downsampled.npy')
saved_path_trigger = os.path.join(script_dir, test, folder, f'triggers_downsampled.npy')
saved_path_piezo = os.path.join(script_dir, test, folder, f'{finger}_downsampled.npy')

# Load the data
piezo_values = np.load(piezo_data_path)  # Finger sensor data
sensor_values = np.load(sensor_values_path)  # Force sensor data
trigger_values = np.load(trigger_data_path)  # Trigger data
 
duration = trigger_values[-1][-1]
print("Duration = ", duration)

fs_trigger =  len(trigger_values[-1])/ duration
print("Trigger freq = ", fs_trigger)
# Original sample rates
fs_sensor = 500  # Hz
fs_piezo = 313  # Hz (approx)
start_time = 7  # seconds
print()
print("sensor duration = ", len(sensor_values)/fs_sensor)
print("piezo duration = ", len(piezo_values)/fs_piezo)
trigger_values = trigger_values[:, int(start_time * fs_trigger):]
sensor_values = sensor_values[int(start_time * fs_sensor):int(duration * 500)]
piezo_values = piezo_values[int(start_time * fs_piezo):int(duration * 313)]
print("new trigger duration = ", len(trigger_values[0])/fs_trigger)
print("new sensor duration = ", len(sensor_values)/fs_sensor)
print("new piezo duration = ", len(piezo_values)/fs_piezo)

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
print(trigger_values_resampled.T.shape, sensor_values_resampled.shape, piezo_values.shape)
# Save the resampled force sensor data as sensor_values_downsapled.npy
np.save(saved_path_sensor, sensor_values_resampled)
np.save(saved_path_trigger, trigger_values_resampled)
np.save(saved_path_piezo, piezo_values)