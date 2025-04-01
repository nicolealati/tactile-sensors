import numpy as np
import matplotlib.pyplot as plt
import os

# Define values
force_values = [2, 4, 6] # N
rise_values = [2, 3, 4]# s
stay_time = [7]
freq_signal = 500

trigger_idle = 0
trigger_force = [e*10  for e in force_values]
trigger_rise =  [e   for e in rise_values]
trigger_fall =  [-e  for e in rise_values]

dir_signal  = r"D:\GitHub\tactile-sensors\interfacce"

if os.name == 'nt':
    os.system('cls')


def GenerateSignal(force_values, rise_values, stay_time, freq = freq_signal):
    
    time_step = 1/freq
    start_time = 0
    signal = []
    time = []
    trigger = []
    for t in range(10*freq):
        signal.append(0)
        trigger.append(trigger_idle)
        time.append(start_time)
        start_time += time_step
     
    for t in stay_time:
        for ind_rise, v_rise in enumerate(rise_values):
            for ind_force, v_force in enumerate(force_values):
                # Calculate the adjusted rise time based on the previous logic
                if ind_rise == 0:
                    rise_time_duration = v_rise*v_force/force_values[0]  # For the first y, use the ratio of current x to first x
                else:
                    rise_time_duration = v_rise*(v_force/force_values[0])  # Scale subsequent x values accordingly
                
                # Idle time before the rise: 5 seconds of 0 intensity
                stay_dur = 7
                idle_time = np.zeros(int(stay_dur*freq))  # 5 seconds of 0
                temp_idle_trigger = [trigger_idle]*int(stay_dur*freq)
                
                # Generate rising phase
                rise_time = np.linspace(0, v_force, int(rise_time_duration*freq))  # Linear rise from 0 to x
                temp_rise_trigger = [trigger_rise[ind_rise]]*int(rise_time_duration*freq)

                # Hold constant phase
                constant_time = np.full(int(t*freq), v_force)  # Hold the value x for z seconds
                temp_const_trigger = [trigger_force[ind_force]]*int(t*freq)
                
                # Generate falling phase
                fall_time = np.linspace(v_force, 0, int(rise_time_duration*freq))  # Linear fall from x to 0
                temp_fall_trigger = [trigger_fall[ind_rise]]*int(rise_time_duration*freq)

                # Concatenate all phases together
                full_signal = np.concatenate([idle_time, rise_time, constant_time, fall_time])
                temp_full_trigger = np.concatenate([temp_idle_trigger, 
                                                    temp_rise_trigger, temp_const_trigger, 
                                                    temp_fall_trigger])
                

                signal.extend(full_signal)
                trigger.extend(temp_full_trigger)           
                time.extend(np.linspace(len(time)*time_step, len(time)*time_step + len(full_signal)*time_step, len(full_signal)))

    signal = np.array(signal)
    trigger = np.array(trigger)
    time = np.array(time)
    
    return time, signal, trigger

# Generate the signal
time, generated_signal, trigger = GenerateSignal(force_values, rise_values, stay_time)


# Scompongo i trigger
trigger_unique = []
trigger_unique.append(trigger_idle)
for c, r, f in zip(trigger_force, trigger_rise, trigger_fall): 
    trigger_unique.extend([r, c, f])
trigger_unique.append(trigger_force[-1])
#print(trigger_unique)
#trigger_unique = [trigger_idle] + trigger_force + [val for pair in zip(trigger_rise, trigger_fall) for val in pair]
trigger_list = [(trigger == value).astype(int) for value in trigger_unique]

plt.plot(time, generated_signal)
plt.show()

# Plot the signal to visualize
#fig, axs = plt.subplots(12, 1)
#for ind_ax, ax in enumerate(axs): 
#    if ind_ax < len(trigger_list):
#        ax.plot(time, trigger_list[ind_ax])
#        ax.plot(time, generated_signal)
#axs[-1].plot(time, generated_signal)

def SaveData(save_dir, name, array_to_save):
    print(save_dir)
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    np.save(f"{save_dir}/{name}", array_to_save)

#SaveData(dir_signal, 'pressure_hfr.npy', [time, generated_signal])

print(time[-1])
