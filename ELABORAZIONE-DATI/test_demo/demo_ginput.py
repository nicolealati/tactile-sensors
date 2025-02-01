import matplotlib.pyplot as plt

# Creazione di un semplice grafico
plt.plot([0, 1, 2, 3], [0, 1, 4, 9], marker='o')
plt.title("Seleziona i punti con il mouse e premi Invio per terminare")

# Utilizzo di ginput per selezionare punti
print("Clicca sui punti desiderati e premi Invio per terminare")
selected_points = plt.ginput(n=-1, timeout=0)

# Visualizzazione dei punti selezionati
print("Punti selezionati:", selected_points)

# Mostra il grafico
plt.show()
