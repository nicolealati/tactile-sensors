import numpy as np
import matplotlib.pyplot as plt
import tkinter as tk

########################
save_fig = 1
random = 0

test = 'pressure'
finger = "little"
n_rec = 2

dir = f"/home/comau/ros/sensor_ws/src/sensor_controller/scripts/records/{test}/{finger}_{n_rec}"

# Tactile sensors
thumb = np.load(dir + "/thumb.npy")
index = np.load(dir + "/index.npy")
middle = np.load(dir + "/middle.npy")
ring = np.load(dir + "/ring.npy")
little = np.load(dir + "/little.npy")

selected_finger = little
other_finger = thumb

y_lim_tactile = 1200
y_lim_max_force = 8
y_lim_min_force = -1

time_diff = 0

res = 50
#scale = 1

########################

fig_name = f"/home/comau/ros/sensor_ws/src/sensor_controller/scripts/records/fig/{test}/{finger}_{n_rec}.png"

ref_signal = np.loadtxt("/home/comau/ros/sensor_ws/src/sensor_controller/scripts/signals/trapezoidal_signal.csv", delimiter=",")

# Force sensor 
sensor_norm = np.load(dir + "/sensor_norm.npy")

sensor_values = np.load(dir + "/sensor_values.npy")
sensor_values_x = sensor_values[:,0]
sensor_values_y = sensor_values[:,1]
sensor_values_z = sensor_values[:,2]

def create_time_vector(fin, freq):
    time_vector = np.arange(0,len(fin))/freq
    return time_vector

freq_tactile = 313
time_tactile = create_time_vector(thumb, freq_tactile) #np.arange(0,len(thumb))/freq_tactile

freq_force = 500
time_force = create_time_vector(sensor_norm, freq_force) #np.arange(0,len(sensor_norm))/freq_force

freq_ref_signal = 250

for i in range (len(sensor_norm)):
    if sensor_norm[i] > 0.07:
        time_diff = time_force[i] - 5
        for j in range (int(time_diff*freq_ref_signal)):
            ref_signal = np.insert(ref_signal, 0, 0)
        break
time_ref_signal = create_time_vector(ref_signal, freq_ref_signal)


def calculate_mean(f):
    mean_taxels = []
    for i in range(len(f)):
        mean_taxels.append(np.mean(f[i, 7]))
    return mean_taxels

root = tk.Tk()
screen_width = root.winfo_screenwidth()
screen_height = root.winfo_screenheight()

# I want to make a big plot with each finger as a sublot in the first column and the second column is the same finger but with the sliding cylinder
fig, axs = plt.subplots(3, 3, figsize = (screen_width/res, screen_height/res))

# Dito usato nel test
axs[0,0].plot(time_tactile, selected_finger)
axs[0,0].set_title(f"Applied pressure ({finger}_{n_rec})")
axs[0,0].set_ylim([-y_lim_tactile,y_lim_tactile])
axs[0,0].set_xlim([0, time_tactile[-1]])

# Altro dito
axs[1,0].plot(time_tactile, other_finger)
axs[1,0].set_title("Pressure not applied (other)")
axs[1,0].set_ylim([-y_lim_tactile,y_lim_tactile])
axs[1,0].set_xlim([0, time_tactile[-1]])

# Norma della forza
axs[0,1].plot(time_force, sensor_norm)
axs[0,1].set_title("Force (NORM)")
axs[0,1].set_xlim([0, time_force[-1]])
axs[0,1].set_ylim([y_lim_min_force, y_lim_max_force])

#axs[1,1].plot(time_tactile, calculate_mean(selected_finger))
#axs[1,1].plot(time_force, scale*sensor_norm)
#axs[1,1].set_title("NORM force vs MEAN tactile")
#axs[1,1].set_xlim([0, time_force[-1]])
#axs[1,1].set_ylim([-y_lim_tactile,y_lim_tactile])

# Ref signal
if not random:
    axs[2,0].plot(time_ref_signal, ref_signal)
    axs[2,0].set_title("Ref signal")
    axs[2,0].set_xlim([0, time_force[-1]])
    axs[2,0].set_ylim([y_lim_min_force, y_lim_max_force])

    axs[1,1].plot(time_ref_signal, ref_signal)
    axs[1,1].plot(time_force, sensor_norm)
    axs[1,1].set_title("Ref signal vs NORM force")
    axs[1,1].set_xlim([0, time_force[-1]])
    axs[1,1].set_ylim([y_lim_min_force, y_lim_max_force])  

    axs[2,1].plot(time_ref_signal, ref_signal)
    axs[2,1].plot(time_force, -sensor_values_z)
    axs[2,1].set_title("Ref signal vs Z force")
    axs[2,1].set_xlim([0, time_force[-1]])
    axs[2,1].set_ylim([y_lim_min_force, y_lim_max_force])

else:
    axs[2,0].axis(False)
    axs[1,1].axis(False)
    axs[2,1].axis(False)

# Componenti della forza
axs[0,2].plot(time_force, sensor_values_x)
axs[1,2].plot(time_force, sensor_values_y)
axs[2,2].plot(time_force, -sensor_values_z)
axs[0,2].set_title("Force (x)")
axs[1,2].set_title("Force (y)")
axs[2,2].set_title("Force (z)")
axs[0,2].set_xlim([0, time_force[-1]])
axs[1,2].set_xlim([0, time_force[-1]])
axs[2,2].set_xlim([0, time_force[-1]])

axs[0,2].set_ylim([y_lim_min_force, y_lim_max_force])
axs[1,2].set_ylim([y_lim_min_force, y_lim_max_force])
axs[2,2].set_ylim([y_lim_min_force, y_lim_max_force])

manager = plt.get_current_fig_manager()
manager.full_screen_toggle()

if save_fig:
    plt.savefig(fig_name, bbox_inches='tight')
    print("Saved figure")

plt.show()

root.quit()