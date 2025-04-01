import numpy as np
import os
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D

os.system('cls')

square = [2.5, 2.5]
rect = [0.1, 0.25]
line = [rect[0], 0.01]

scaled_force = rect[1]

fig, ax = plt.subplots(figsize=(6, 6))
# ax.add_patch(Rectangle((-square[0] / 2, -square[1] / 2), square[0], square[1], color='grey', alpha=0.2))
# ax.add_patch(Rectangle((-rect[0], -rect[1]), rect[0] * 2, rect[1] * 2, color='green'))
# ax.add_patch(Rectangle((-1.5 * line[0] - line[0] / 2, 0), line[0] * 4, line[1], color='black'))
ax.add_patch(Rectangle((-rect[0] / 2, -rect[1] / 2), rect[0], rect[1], color='red', alpha=0.9))

ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.axis(False)
ax.grid(False)

plt.show()