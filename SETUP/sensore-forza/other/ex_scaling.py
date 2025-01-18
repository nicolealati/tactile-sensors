import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import os

if os.name == 'nt':
    os.system('cls')

tolerance = 0.30
required_force = 6

height_rect = 0.1

n_frame = 10000
force_x = np.linspace(-2*np.pi, 2*np.pi, n_frame)-n_frame/2

'''
applied_force = required_force+np.sin(force_x)
'''

A = (6.4-4)/2 
B = 2*np.pi/10
C = np.random.uniform(0, 2*np.pi)  
f = lambda x: 6+A*np.sin(B*x+C)
applied_force = f(force_x)
applied_force = np.clip(applied_force, 5.5, 7.2)


trans_force = applied_force-required_force
scaled_force = trans_force*height_rect/tolerance

# Creazione del grafico
fig, axs = plt.subplots(1, 2, figsize=(10, 5))

axs[0].axhline(-tolerance, color='red')
axs[0].axhline(tolerance, color='red')
axs[0].plot(force_x, trans_force, color='black')
axs[0].set_title('Netwon')

axs[1].axhline(-height_rect, color='red')
axs[1].axhline(height_rect, color='red')
axs[1].plot(force_x, scaled_force, color='black')
axs[1].set_title('Scaled')

for ax in axs: 
    ax.grid(True)
    ax.set_ylim([-1.5, 1.5])

plt.show()