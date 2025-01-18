import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import os

if os.name == 'nt':
    os.system('cls')

# Creazione del grafico
fig, ax = plt.subplots()

# Definizione del rettangolo (centrato in (0,0), dimensioni 2x3)
rectangle = Rectangle((-1, -1.5), 2, 3, edgecolor='blue', facecolor='lightblue')

# Aggiunta del rettangolo al grafico
ax.add_patch(rectangle)

# Impostazione degli assi
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)

# Titolo e aspetto del grafico
ax.set_title("Rettangolo 2x3 centrato in (0,0)")
ax.set_aspect('equal', adjustable='box')
ax.grid(True)

# Visualizzazione del grafico
plt.show()