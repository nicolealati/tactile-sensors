import numpy as np
import matplotlib.pyplot as plt
import os
from matplotlib.animation import FuncAnimation
import time


def CleanTerminal():
    if os.name == 'nt':
        _ = os.system('cls')

CleanTerminal()

## Creo l'ellisse
a, b = 45, 45  # Semiassi lungo x e y
theta = np.linspace(0, 2 * np.pi, 1000)  # Generazione della griglia per l'ellisse

x = a * np.cos(theta)
y = b * np.sin(theta)

# Creazione della figura 2D
fig, ax = plt.subplots(figsize=(8, 8))
ax.plot(x, y, color='lightblue')
ax.fill(x, y, color='lightblue', alpha=0.5)

hg = 0
vg = 0

# Funzione di aggiornamento per l'animazione
start_time = time.time()  # Tempo iniziale per il cronometro

def update(frame):
    elapsed_time = time.time() - start_time  # Calcola il tempo trascorso
    ax.clear()
    ax.plot(x, y, color='lightblue')
    ax.fill(x, y, color='lightblue', alpha=0.5)
    ax.axhline(hg, color='red', linewidth=1, linestyle='-')
    ax.axvline(frame, color='red', linewidth=1, linestyle='-')
    ax.text(-52, hg, f'{hg}°', color='red', fontsize=10, ha='left', va='center')  # Vicino alla barra verticale
    ax.text(frame, -52, f'{vg}°', color='red', fontsize=10, ha='center', va='bottom')  # Vicino alla barra orizzontale
    ax.text(40, -50, f"Time: {elapsed_time:.2f}s", color='black', fontsize=12, ha='right', va='bottom')  # Cronometro in basso a destra
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title("Rotolamento")
    ax.axis(False)
    ax.grid(True)

    # Ferma l'animazione dopo l'ultimo fotogramma
    if frame >= 45:
        ani.event_source.stop()

# Creazione dell'animazione
ani = FuncAnimation(fig, update, frames=np.linspace(0, 45, 100), interval=20)

plt.show()

