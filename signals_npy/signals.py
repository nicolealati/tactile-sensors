import numpy as np
import matplotlib.pyplot as plt

#load signal from C:\Users\lar\Documents\Signals\Cube sliding
signal_path = 'C:/Users/lar/Documents/Signals/Cube stationary/'
thumb = np.load(signal_path + 'thumb.npy')
index = np.load(signal_path + 'index.npy')
middle = np.load(signal_path + 'middle.npy')
ring = np.load(signal_path + 'ring.npy')
little = np.load(signal_path + 'little.npy')
finger = thumb
taxel = 2
freq = 310

#i want to sum instant by instant all texels of the finger
summed_signal = np.zeros(finger.shape[0])
for i in range(finger.shape[0]):
    summed_signal[i] = np.sum(finger[i])

integral_finger = np.zeros(finger.shape[0])
for i in range(1,finger.shape[0]):
    integral_finger[i] = integral_finger[i-1] + finger[i,taxel]/freq

integral_summed = np.zeros(finger.shape[0])
for i in range(1,finger.shape[0]):
    integral_summed[i] = integral_summed[i-1] + summed_signal[i]/freq

#plot signal of finger and summed with legend
plt.plot(finger[:,taxel])
plt.plot(summed_signal)
plt.plot(integral_finger)
plt.plot(integral_summed)
plt.legend(['thumb', 'summed', 'integral', 'integral summed'])
plt.show()

