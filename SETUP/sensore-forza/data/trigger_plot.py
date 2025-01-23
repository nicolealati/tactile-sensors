import numpy as np
import matplotlib.pyplot as plt
import tkinter as tk
import os

if os.name == 'nt':
    os.system('cls')

test = "rolling"

dir = "data/trigger_"
t =        np.load(f"{dir}{test}/time_interp.npy")
x =        np.load(f"{dir}{test}/x_interp.npy")
y =        np.load(f"{dir}{test}/y_interp.npy")
stop =     np.load(f"{dir}{test}/trigger_stop.npy")
destra =   np.load(f"{dir}{test}/trigger_destra.npy")
sinistra = np.load(f"{dir}{test}/trigger_sinistra.npy")
alto =     np.load(f"{dir}{test}/trigger_alto.npy")
basso =    np.load(f"{dir}{test}/trigger_basso.npy")

if test == "sliding":
    line = 7  
    a_1 =          np.load(f"{dir}{test}/angle_interp.npy")
    orario_1 =     np.load(f"{dir}{test}/trigger_orario.npy")
    antiorario_1 = np.load(f"{dir}{test}/trigger_antiorario.npy")


if test == "rolling":
    line = 9 
    a_1 =          np.load(f"{dir}{test}/angle_interp_1.npy")
    a_2 =          np.load(f"{dir}{test}/angle_interp_2.npy")
    orario_1 =     np.load(f"{dir}{test}/trigger_orario_1.npy")
    antiorario_1 = np.load(f"{dir}{test}/trigger_antiorario_1.npy")
    orario_2 =     np.load(f"{dir}{test}/trigger_orario_2.npy")
    antiorario_2 = np.load(f"{dir}{test}/trigger_antiorario_2.npy")

fig, axs = plt.subplots(line,2, figsize=(12, 8))

axs[0,0].plot(t, stop),         axs[0,0].legend("STOP"),         axs[0,0].set_ylim([-0.5, 1.5])
axs[1,0].plot(t, destra),       axs[1,0].legend("DESTRA"),       axs[1,0].set_ylim([-0.5, 1.5])
axs[2,0].plot(t, sinistra),     axs[2,0].legend("SINISTRA"),     axs[2,0].set_ylim([-0.5, 1.5])
axs[3,0].plot(t, alto),         axs[3,0].legend("ALTO"),         axs[3,0].set_ylim([-0.5, 1.5])
axs[4,0].plot(t, basso),        axs[4,0].legend("BASSO"),        axs[4,0].set_ylim([-0.5, 1.5])
axs[5,0].plot(t, orario_1),     axs[5,0].legend("ORARIO - 1"),   axs[5,0].set_ylim([-0.5, 1.5])
axs[6,0].plot(t, antiorario_1), axs[6,0].legend("ANTI - 1"),     axs[6,0].set_ylim([-0.5, 1.5])
if test == "rolling":
    axs[7,0].plot(t, orario_2), axs[7,0].legend("ORARIO - 2"),   axs[7,0].set_ylim([-0.5, 1.5])
    axs[8,0].plot(t, antiorario_2), axs[8,0].legend("ANTI - 2"), axs[8,0].set_ylim([-0.5, 1.5])

axs[0,1].axis(False)
axs[1,1].axis(False)
axs[2,1].plot(t, x), axs[2,1].legend("X")
axs[3,1].plot(t, y), axs[3,1].legend("Y")
axs[4,1].plot(t, a_1), axs[4,1].legend("A")
axs[6,1].axis(False)
if test == "rolling":
    axs[5,1].plot(t, a_2), axs[4,1].legend("A")
    axs[7,1].axis(False)
    axs[8,1].axis(False)
else: 
    axs[5,1].axis(False)


    

plt.show()