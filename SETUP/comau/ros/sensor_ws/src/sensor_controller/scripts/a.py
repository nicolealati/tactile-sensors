import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter

taxel = 2
dir = "/home/alex/ros/Sensor_ws/src/sensor_controller/scripts/records/PushtoThumb"
thumb = np.load(dir + "/thumb.npy")
index = np.load(dir + "/index.npy")
middle = np.load(dir + "/middle.npy")
ring = np.load(dir + "/ring.npy")
little = np.load(dir + "/little.npy")

finger=thumb

#voglio creare un filtro passa basso per filtrare il segnale
# def butter_lowpass(cutoff, fs, order=5):
#     nyquist = 0.5 * fs
#     normal_cutoff = cutoff / nyquist
#     b, a = butter(order, normal_cutoff, btype='low', analog=False)
#     return b, a

# def butter_lowpass_filter(data, cutoff, fs, order=5):
#     b, a = butter_lowpass(cutoff, fs, order=order)
#     y = lfilter(b, a, data)
#     return y

# #filtro il segnale
# fs = 310
# cutoff = 50
# order = 5
# new_finger = butter_lowpass_filter(finger, cutoff, fs, order)

#filtro passa alto
def butter_highpass(cutoff, fs, order=5):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    return b, a

def butter_highpass_filter(data, cutoff, fs, order=5):
    b, a = butter_highpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y

#filtro il segnale
fs = 310
cutoff = 30
order = 2
new_finger = butter_highpass_filter(finger, cutoff, fs, order)
#rms of the signal in a 100 window

#otteniamo l'integrale del segnale filtrato
freq = 25
integral = np.zeros(finger.shape)
integral[0] = new_finger[0]/freq
for i in range(1, new_finger.shape[0]):
    integral[i] = integral[i-1] + new_finger[i]/freq
    if integral[i,taxel] < 0:
       integral[i,taxel] = 0

# #i want to subract the mean after every 100 samples
# for i in range(0, integral.shape[0], 1000):
#     mean = np.mean(integral[i:i+1000], axis=0)
#     integral[i:i+1000] -= mean
# for i in range(1, new_finger.shape[0]):
#     integral[i,taxel] = abs(integral[i,taxel])
        
    # for j in range(1, integral[i].shape[0]):
    #     if integral[i][j] < 0:
    #         integral[i][j] = 0

plt.figure()
plt.plot(finger[:,taxel], alpha=0.5)
plt.plot(integral[:,taxel])
plt.xlabel("Samples")
plt.ylabel("Magnitude")
plt.title("FInger signal integral")
plt.grid()
plt.legend(["Original", "Filtered", "Integral"])
plt.show()
