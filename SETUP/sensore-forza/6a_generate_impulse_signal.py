import numpy as np
import matplotlib.pyplot as plt
import os 


dir_signal  = r"D:\GitHub\tactile-sensors\SETUP\sensore-forza\sequences\impulse"
dir_trigger = r"D:\GitHub\tactile-sensors\SETUP\sensore-forza\sequences\impulse\trigger_impulse"

if os.name == 'nt':
            os.system('cls')

def AccPhase(impulse_times, time, start_time, intervals, end_time): 
    for interval in intervals:
        if time >= end_time:  # 30 seconds from the previous end time
            break
        impulse_times.append(time)
        time += interval
    return  time, impulse_times
     
def ConstPhase(impulse_times, time, start_time, interval, end_time): 
    while time < end_time:
        impulse_times.append(time)
        time += interval
    return time, impulse_times
    
def GenerateImpulseTimes():
    impulse_times = []
    trigger_times = [[],  # stop
                     [], [], #rate 1, acc 1
                     [], [], #rate 2, acc 2
                     [], # rate 3
                     []] # stop
    phase_end_times=[]

    # STOP
    current_time = 10
    trigger_times[0] = [0, current_time]
    phase_end_times.append(current_time)

    #print(trigger_times)
    #print(phase_end_times)

    # RATE 1   
    prev_end_time = current_time
    end_time = prev_end_time+30
    interval = 1.5
    current_time, impulse_times = ConstPhase(impulse_times, current_time, prev_end_time, interval, end_time)
    '''while time < end_time:
        impulse_times.append(time)
        time += interval
    phase1_end = time'''
    trigger_times[1] = [float(prev_end_time), float(current_time)]
    phase_end_times.append(float(current_time))
    #print(trigger_times)
    #print(phase_end_times)

    '''# 1ST PHASE
    interval = 1.5
    end_time = 30
    while time < end_time:
        impulse_times.append(time)
        time += interval
    phase1_end = time'''
    
    # ACC 1: reduce interval
    intervals = np.arange(1.45, 1.0-0.05, -0.05)
    prev_end_time = current_time
    current_time, impulse_times = AccPhase(impulse_times, current_time, prev_end_time, intervals, prev_end_time+60)
    ''' for interval in intervals:
        if time >= 60:  # 30 seconds from the previous end time
            break
        impulse_times.append(time)
        time += interval
    phase2_end = time'''
    trigger_times[2] = [float(prev_end_time), float(current_time)]
    phase_end_times.append(float(current_time))
    #print(trigger_times)
    #print(phase_end_times)

    # RATE 2
    prev_end_time = current_time
    end_time = prev_end_time+30
    interval = 1.0
    current_time, impulse_times = ConstPhase(impulse_times, current_time, prev_end_time, interval, end_time)
    '''while time < end_time:
        impulse_times.append(time)
        time += interval
    phase3_end = time'''
    trigger_times[3] = [float(prev_end_time), float(current_time)]
    phase_end_times.append(float(current_time))
    #print(trigger_times)
    #print(phase_end_times)
    
    # ACC 2: reduce interval
    intervals = np.arange(0.95, 0.5-0.05, -0.05)
    prev_end_time = current_time
    current_time, impulse_times = AccPhase(impulse_times, current_time, prev_end_time, intervals, prev_end_time+60)
    '''for interval in intervals:
        if current_time >= end_time+30:  # Another 30 seconds
            break
        impulse_times.append(current_time)
        current_time += interval
    phase4_end = current_time'''
    trigger_times[4] = [float(prev_end_time), float(current_time)]
    phase_end_times.append(float(current_time))
    #print(trigger_times)
    #print(phase_end_times)
    
    # RATE 3
    prev_end_time = current_time
    end_time = prev_end_time+30
    interval = 0.5
    '''while current_time < end_time:
        impulse_times.append(current_time)
        current_time += interval
    phase5_end = current_time'''
    current_time, impulse_times = ConstPhase(impulse_times, current_time, prev_end_time, interval, end_time)
    trigger_times[5] = [float(prev_end_time), float(current_time)]
    phase_end_times.append(float(current_time))
    #print(trigger_times)
    #print(phase_end_times)

    # STOP 
    prev_end_time = current_time
    end_time = prev_end_time+10
    trigger_times[6] = [float(prev_end_time), float(end_time+10)]
    phase_end_times.append(float(current_time))
    print(trigger_times)
    print(phase_end_times)

    
    return impulse_times, trigger_times, phase_end_times

def GenerateImpulse(impulse_times, max_amplitude=8, freq=500):
   
    #time_step = 1/freq
    
    # Create the signal array initialized to 0
    total_duration = int(np.ceil(max(impulse_times))+10)  # Duration based on the latest impulse time
    signal = np.zeros(int(total_duration*freq))  # Initialize signal array with zeros
    
    # Create a time array corresponding to the signal
    time = np.linspace(0, total_duration, len(signal))
    
    # Iterate over each impulse time and set the corresponding signal value
    for impulse_time in impulse_times:
        # Find the index in the signal array corresponding to the impulse time
        index = int(np.round(impulse_time*freq))
        if 0 <= index < len(signal):
            signal[index] = max_amplitude  # Set the impulse value
    
    # Return the time and signal arrays
    return time, signal

# Generate the impulse times and phase end times
impulse_times, trigger_times, phase_end_times = GenerateImpulseTimes()

# Generate the impulse signal
time, generated_signal = GenerateImpulse(impulse_times)


# Generate trigger
trigger_list = [[],  # stop
                [], [], #rate 1, acc 1
                [], [], #rate 2, acc 2
                [] ]# rate 3
                # stop'''

for ind in range(1, len(trigger_list)): 
    mask = (time >= trigger_times[ind][0]) & (time <= trigger_times[ind][1])
    zeros = np.zeros(len(time)) 
    zeros[mask] = 1
    trigger_list[ind] = zeros

mask1 = (time >= trigger_times[0][0]) & (time <= trigger_times[0][1])
mask2 = (time >= trigger_times[6][0]) & (time <= trigger_times[6][1])
mask = mask1+ mask2
zeros = np.zeros(len(time)) 
zeros[mask] = 1
trigger_list[0] = zeros

# Plot the signal to visualize
plt.plot(time, generated_signal)
plt.title("Variable Impulse Signal")
plt.grid(True)
plt.xlabel("Time [s]")
plt.ylabel("Intensity")
# Add red vertical lines to divide the phases
for phase_end_time in phase_end_times:
    plt.axvline(x=phase_end_time, color='red', linestyle='--', linewidth=2)

fig, axs = plt.subplots(7, 1)
for ind_ax, ax in enumerate(axs): 
    if ind_ax < 6:
        ax.plot(time, trigger_list[ind_ax])
        #ax.plot(time, generated_signal)
axs[-1].plot(time, generated_signal)
for phase_end_time in phase_end_times:
    axs[-1].axvline(x=phase_end_time, color='red', linestyle='--', linewidth=2)
plt.show()

def SaveData(save_dir, name, array_to_save):
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    np.save(f"{save_dir}\{name}", array_to_save)

SaveData(dir_signal, "impulse_sequence.npy", [time, generated_signal])
SaveData(dir_trigger, "time.npy", time)
SaveData(dir_trigger, "trigger_stop.npy", trigger_list[0])
SaveData(dir_trigger, "trigger_rate_1.npy", trigger_list[1])
SaveData(dir_trigger, "trigger_acc_1.npy", trigger_list[2])
SaveData(dir_trigger, "trigger_rate_2.npy", trigger_list[3])
SaveData(dir_trigger, "trigger_acc_2.npy", trigger_list[4])
SaveData(dir_trigger, "trigger_rate_3.npy", trigger_list[5])
