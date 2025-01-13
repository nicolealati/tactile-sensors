import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import os

def CleanTerminal():
    if os.name == 'nt':
        _ = os.system('cls')
CleanTerminal()

## Define the time and positions for the axes
#t = np.array([0, 1, 3, 5, 6, 8, 15])  # Time steps
#x = np.array([0, 0.5, 0.5, 1, 0, 1, 0])  # Position of the y-axis (X coordinate)
#y = np.array([0, 0, 1, 1, 0.5, 0.5, 0])  # Position of the x-axis (Y coordinate)
#
##### Smooth movements
#
## Ellipse parameters: Scale it to fit between -1 and 1
#a, b = 1, 1  # Semi-major and semi-minor axes set to 1 for a unit circle
#theta = np.linspace(0, 2 * np.pi, 1000)
#ellipse_x = a * np.cos(theta)
#ellipse_y = b * np.sin(theta)
#
#t_start = time.start()
#
## Create the figure
#fig, ax = plt.subplots(figsize=(8, 8))
#ax.plot(ellipse_x, ellipse_y, color='lightblue')
#ax.fill(ellipse_x, ellipse_y, color='lightblue', alpha=0.5)
#
## Set labels and title
#ax.set_xlabel("X")
#ax.set_ylabel("Y")
#ax.set_title("Axis Movement Animation")
#
#ax.grid(False)
#ax.axis(False)

#plt.show()



def plot_animation():

    start_time = time.time()  # Tempo iniziale per il cronometro
    
    fig, ax = plt.subplots(figsize=(8, 8))
    x = np.linspace(0, 2 * np.pi, 100)
    line = ax.plot(x, np.sin(x))

   
    # Numero totale di frame
    total_frames = 100
    frame_rate = 10  # 10 frame al secondo
    frame_interval = 1 / frame_rate  # intervallo in secondi

        # Loop per aggiornare i frame
    for i in range(total_frames):

        ax.clear()

        # Configurazione iniziale del grafico
        fig, ax = plt.subplots()
        x = np.linspace(0, 2 * np.pi, 100)
        line, = ax.plot(x, np.sin(x))

        # Impostare i limiti del grafico
        ax.set_ylim(-1.5, 1.5)
    

        # Aggiornamento dei dati del grafico
        line.set_ydata(np.sin(x + i * 0.1))

        # Redraw del grafico
        plt.pause(0.001)  # per aggiornare la visualizzazione
        time.sleep(frame_interval - 0.001)  # per garantire 10 FPS
        elapsed_time = time.time() - start_time  # Calcola il tempo trascorso
        ax.text(5, 1.3, f"Time: {elapsed_time:.2f}s", color='black', fontsize=12, ha='right', va='bottom')  # Cronometro in basso a destra

        if elapsed_time >= 1:
            break

    plt.show()

# Esegui l'animazione
plot_animation()







