import matplotlib.pyplot as plt
import numpy as np
import time

def plot_animation():
    # Configurazione iniziale del grafico
    fig, ax = plt.subplots()
    x = np.linspace(0, 2 * np.pi, 100)
    line, = ax.plot(x, np.sin(x))

    # Impostare i limiti del grafico
    ax.set_ylim(-1.5, 1.5)

    # Numero totale di frame
    total_frames = 500
    frame_rate = 100  # 100 frame al secondo
    frame_interval = 1 / frame_rate  # intervallo in secondi

    # Loop per aggiornare i frame
    for i in range(total_frames):
        # Aggiornamento dei dati del grafico
        line.set_ydata(np.sin(x + i * 0.1))

        # Redraw del grafico
        plt.pause(0.001)  # per aggiornare la visualizzazione
        time.sleep(frame_interval - 0.001)  # per garantire 10 FPS

    plt.show()

# Esegui l'animazione
plot_animation()
