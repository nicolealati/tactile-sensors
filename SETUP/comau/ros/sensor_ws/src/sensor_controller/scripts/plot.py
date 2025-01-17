#plot /home/alex/ros/Sensor_ws/src/sensor_controller/scripts/records/21/filtered_thumb.npy, it's a nx8 array

from matplotlib import pyplot as plt

import numpy as np

data = np.load("/home/alex/ros/Sensor_ws/src/sensor_controller/scripts/records/22/thumb.npy")
#subplot
fig, axs = plt.subplots(4, 2)

for i in range(8):
    axs[i//2, i%2].plot(data[:, i])
    axs[i//2, i%2].set_title(f"Taxel {i}")
plt.show()


