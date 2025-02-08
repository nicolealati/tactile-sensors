import numpy as np
import matplotlib.pyplot as plt
import os

if os.name == 'nt':
    os.system('cls')

# Load the sensor data
test = "pressure"
fingers = ["thumb", "index"]
n_reps = [1, 2]

max_values = []
min_values = []
mean_values = []
std_values = []

all_values = []

dir = rf"D:\GitHub\tactile-sensors\ELABORAZIONE-DATI\recorded-data\rec_03_02"

def ExtractValues(time_vector, signal_vector, mask): 
    values = []
    times = []
  
    valid_indices = []
    for i, t in enumerate(mask): 
        if t == 1:
            valid_indices.append(i)
        else: 
            valid_indices = valid_indices[500:-500]
            for i in valid_indices:
                values.append(signal_vector[i])
                times.append(time_vector[i])
            valid_indices = []
    return values, times


for finger in fingers:
    for n_rep in n_reps:
        folder = f"{test}_{finger}_{n_rep}"
        filepath = f"{dir}\{test}\{folder}"

        sensor_data = np.load(f"{filepath}\sensor_data.npy")
        z_time = np.load(f"{filepath}\sensor_time.npy")
        z_data = [-row[2] for row in sensor_data]
        received_triggers = np.load(rf"{filepath}\received_trigger.npy")
        trigger_time = received_triggers[-1]
        trigger_stop = received_triggers[0]

        original_ind = np.linspace(0, 1, len(trigger_time))
        new_ind = np.linspace(0, 1, len(z_time))
        trigger_time_intep = np.interp(new_ind, original_ind, trigger_time)
        trigger_stop_intep = np.interp(new_ind, original_ind, trigger_stop)
               
        extracted_values, extracted_time = ExtractValues(z_time, z_data, trigger_stop_intep)

        plt.plot(z_time, z_data, color = 'black')
        plt.plot(trigger_time_intep, trigger_stop_intep, color = 'red')
        plt.plot(extracted_time, extracted_values, '*', color = 'blue')
        plt.title(f"pressure_{finger}_{n_rep}")
        #plt.show()
        
        dir_save = rf"D:\GitHub\tactile-sensors\ELABORAZIONE-DATI\define_z_thresholds\fig"
        if not os.path.exists(dir_save):
                os.makedirs(dir_save)


        max_values.append(max(extracted_values))
        min_values.append(min(extracted_values))
        mean_values.append(np.mean(extracted_values))
        std_values.append(np.std(extracted_values))

        all_values.extend(extracted_values)

print("Max value = ", max(max_values))
print("Min value = ", min(min_values))
print("Mean value = ", np.mean(mean_values))
print("Std value = ", np.std(std_values))
print()

print("CONCATENATION")
print("Max value = ", max(all_values))
print("Min value = ", min(all_values))
print("Mean value = ", np.mean(all_values))
print("Std value = ", np.std(all_values))

print("THRESHOLD = ", np.mean(all_values)+2*np.std(all_values))














'''
    step = 0.05
    threshold_values = np.arange(0, 0.5+step, step)

    def BandeVerticali(times, values, threshold): 
        in_band = False
        start = 0
        for i in range(len(values)):
            if values[i] > threshold and not in_band:
                start = times[i]
                in_band = True
            elif values[i] > threshold and in_band:
                plt.axvspan(start, times[i], color='red', alpha=0.3)
                in_band = False
        if in_band:
            plt.axvspan(start, times[-1], color='red', alpha=0.3)

    BandeVerticali(time, z_data, threshold=3)

    plt.plot(time, z_data, color = 'black')
    plt.xlim([0, time[-1]])
    plt.title(3)
    plt.show()
    #for th in threshold_values:
        





'''