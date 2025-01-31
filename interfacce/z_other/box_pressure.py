import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os
import numpy as np
import time
from matplotlib.animation import FuncAnimation

def CleanTerminal():
    if os.name == 'nt':
        _ = os.system('cls')
CleanTerminal()

n = 1000 # numero frame 
t = np.array([0, 4])  # intervallo di tempo
time_interpolated = np.linspace(0, t[-1], n)  # Tempo interpolato
x = np.linspace(0, 2 * np.pi, n) # vettore x
ampiezza = (np.cos(x)+1)/2 # Ampiezza variabile tra 0 e 1
current_force = np.abs(np.sin(x)*ampiezza) # Sinusoide con ampiezza modulata
# np.round(np.random.rand(n),2) # Valori di forza casuali

# Dimensioni del rettangolo
width = 0.1
height = 1.0
required_values = 0.5
tolerance = 0.2*required_values 

# Creazione della figura e dell'asse
fig, ax = plt.subplots()

# Tempo di inizio dell'animazione
start_time = time.time()

# Funzione per animare 
def update(frame):
    ax.clear()

    # Tempo trascorso dall'inizio dell'animazine
    elapsed_real_time = time.time() - start_time

    # Frame più vicino in base al tempo reale trascorso
    frame = np.searchsorted(time_interpolated, elapsed_real_time, side='right')-1

    # Banda verde di tolleranza
    band = patches.Rectangle((-width/2, required_values-tolerance), width, 2*tolerance, edgecolor='none', facecolor='green', alpha = 0.4)
    ax.add_patch(band)
    
    # Valore esatto voluto  
    ax.plot([-width/2, width/2], [required_values, required_values], color='red', linewidth=2)

    # Valore corrente
    rect_current = patches.Rectangle((-width/2, width/2), width, current_force[frame], linewidth=2, edgecolor='lightblue', facecolor='lightblue', alpha=0.7)
    ax.add_patch(rect_current)

    # Rettangolo bianco
    rect = patches.Rectangle((-width/2, width/2), width, height, linewidth=2, edgecolor='black', facecolor='none')
    ax.add_patch(rect)

    # Impostazione dei limiti dell'asse
    ax.set_xlim(-0.5, 0.5)
    ax.set_ylim(-0.1, 1.1)
    ax.set_aspect('equal') 
    ax.axis(True)
    ax.grid(False)
    plt.title('Test')

    # Interrompe l'animazione quando si raggiunge l'ultimo frame
    if frame >= len(time_interpolated) - 1:
        ani.event_source.stop()
        plt.close()
        return

# Visualizzazione del grafico
ani = FuncAnimation(fig, update, interval=10, cache_frame_data=False)  # Intervallo breve per controllare gli aggiornamenti di frame frequentemente
plt.show()


