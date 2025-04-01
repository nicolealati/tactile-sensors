import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import os

os.system('cls')

finger="index"
test_n = 2
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

if test == "pressure":
    
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
        if sensor_values[i][2] < 0.2: #THRESHOLD
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

else: 
    # Define the relative path to the file
    piezo_data_path =    os.path.join(script_dir, "data", "downsampled_data", folder, f'piezo_{finger}_downsampled.npy')
    trigger_data_path =  os.path.join(script_dir, "data", "downsampled_data", folder, 'triggers_downsampled.npy')
    sensor_values_path = os.path.join(script_dir, "data", "downsampled_data", folder, 'sensor_data_downsampled.npy')

    # Load the data
    piezo_values = np.load(piezo_data_path)  # Finger sensor data
    sensor_values = np.load(sensor_values_path)  # Force sensor data
    trigger_values = np.load(trigger_data_path)  # Trigger data

    # Ignore the last row (time values)
    trigger_values_data = trigger_values[:-1, :]

    # Find indices where values change from 0 to 1
    diff = np.diff(trigger_values_data, axis=1)

    # Find indices where the difference is non-zero --> cambio task 
    change_indices = np.where(diff != 0)

    # Adjust the column indices to match the original array
    #change_indices = (change_indices[0], change_indices[1]+1)

    # Flatten indices into a single array
    all_change_indices = np.sort(np.unique(change_indices[1]+1))
    all_change_indices = np.concatenate(([0], all_change_indices, [len(trigger_values_data.T)]))

    # Keep indices where the difference is greater than 1 --> rimuovo indici consecutivi
    diffs = np.diff(all_change_indices)
    all_change_indices = all_change_indices[np.concatenate(([True], diffs > 1))]

    # Print results
    print("All change indices:", all_change_indices)

    trigger_values_cleaned = []
    sensor_values_cleaned = []
    piezo_values_cleaned = []

    for i in range(len(all_change_indices)-1):
        start_idx = all_change_indices[i]
        end_idx = all_change_indices[i+1]

        # Calculate the window size and the slicing indices
        window_size = end_idx - start_idx
        start_cut = start_idx + window_size//6*2   # Skip the first 2/6
        end_cut = end_idx - window_size//6         # Skip the last 1/6

        # Extract the cleaned segment
        trigger_values_cleaned.append(trigger_values[:, start_cut:end_cut])
        sensor_values_cleaned.append(sensor_values[start_cut:end_cut])
        piezo_values_cleaned.append(piezo_values[start_cut:end_cut])

    # Combine all cleaned segments
    trigger_values_cleaned = np.concatenate(trigger_values_cleaned, axis=1)
    sensor_values_cleaned = np.concatenate(sensor_values_cleaned, axis=0)
    piezo_values_cleaned = np.concatenate(piezo_values_cleaned, axis=0)

    # Print the cleaned msg data
    print(f"\nInitial trigger data shape: {trigger_values.shape}")
    print(f"Cleaned trigger data shape: {trigger_values_cleaned.shape}")

    # Optional: plot the cleaned data
    plt.figure(figsize=(10, 6))
    plt.plot(trigger_values_cleaned.T)
    plt.title('Cleaned Message')
    #plt.show()

    print()
    print(f"Cleaned sensor values shape: {sensor_values_cleaned.shape}")
    print(f"Cleaned piezo values shape: {piezo_values_cleaned.shape}")

    labels = np.full(trigger_values_cleaned.shape[1], 0, dtype=int)
    num_unlabeled = 0

    # Assign labels based on which row in trigger_values_cleaned is active
    for i in range(trigger_values_cleaned.shape[1]):  # Iterate over time steps
        active_indices = np.where(trigger_values_cleaned[:, i] == 1)[0]  # Find active trigger(s)
        if len(active_indices) > 0:
            labels[i] = active_indices[0]  # Assign the first active trigger index
        else:
            labels[i] = labels[i-1]  # Assign the previous label
            num_unlabeled += 1

    print(f"\nNumber of unlabeled time steps: {num_unlabeled}")

    # Optional: Plot the labels
    # Assuming 'labels' and 'trigger_values_cleaned.T[-1]' are already defined
    plt.figure(figsize=(10, 3))
    # Plot the 'labels' data
    plt.plot(labels, marker='o', linestyle='None', markersize=3, label="Labels")
    # Plot the 'trigger_values_cleaned.T[-1]' on the same axis
    plt.plot(trigger_values_cleaned[-1], marker='x', linestyle='-', markersize=3, label="Trigger Values")
    plt.title("Labels and Trigger Values over Time")
    plt.xlabel("Time Step")
    plt.ylabel("Trigger Index")
    plt.legend()
    plt.grid()
    #plt.show()

    # Save the labels
    create_folder(os.path.join(script_dir, "data", "labeled_data", folder))

    labels_path = os.path.join(script_dir, "data", "labeled_data", folder, 'labels.npy')
    np.save(labels_path, labels)

    # Save the cleaned data
    saved_path_sensor = os.path.join(script_dir, "data", "labeled_data", folder, 'sensor_data_labeled.npy')
    saved_path_piezo =  os.path.join(script_dir, "data", "labeled_data", folder, f'piezo_{finger}_labeled.npy')

    np.save(saved_path_sensor, sensor_values_cleaned)
    np.save(saved_path_piezo, piezo_values_cleaned)

    print()