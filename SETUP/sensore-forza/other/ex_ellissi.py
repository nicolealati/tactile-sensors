import matplotlib.pyplot as plt
import numpy as np
import os

def CleanTerminal():
    if os.name == 'nt':
        _ = os.system('cls')
CleanTerminal()

def ellipse(a, b):
    theta = np.linspace(0, 2 * np.pi, 500)
    ellipse_x = a * np.cos(theta)
    ellipse_y = b * np.sin(theta)
    return ellipse_x, ellipse_y

e_x_1, e_y_1 = ellipse(0.5, 0.5)
e_x_2, e_y_2 = ellipse(1, 0.5)

# Create subplots
fig, axs = plt.subplots(1, 2, figsize=(10, 5))

def create_subplot(e_x, e_y, n_subplot, title): 
    axs[n_subplot].plot(e_x, e_y, color='pink')
    axs[n_subplot].fill(e_x, e_y, color='pink', alpha=0.8)
    scale = 1.3
    axs[n_subplot].plot([-scale*e_x, scale*e_x], [0, 0], color='red', linewidth=2, linestyle='-')   
    #axs[n_subplot].axhline(0, color='red', linewidth=2, linestyle='-')
    axs[n_subplot].set_title(title)
    axs[n_subplot].set_xlim(-1.5, 1.5)
    axs[n_subplot].set_ylim(-1.5, 1.5)
    axs[n_subplot].axis(False)
    axs[n_subplot].grid(False)   

create_subplot(e_x_1, e_y_1, 0, "Rotolamento 1 - ML")
create_subplot(e_x_2, e_y_2, 1, "Rotolamento 2 - AP")

plt.tight_layout()
plt.show()
