import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter
from scipy.signal import freqs

taxel = 2
dir = "/home/alex/ros/Sensor_ws/src/sensor_controller/scripts/records/PushtoThumb"
thumb = np.load(dir + "/thumb.npy")
index = np.load(dir + "/index.npy")
middle = np.load(dir + "/middle.npy")
ring = np.load(dir + "/ring.npy")
little = np.load(dir + "/little.npy")

finger=thumb

#detrend lineare del segnale
x = np.arange(len(finger[:,taxel]))
z = np.polyfit(x, finger[:,taxel], 2)
p = np.poly1d(z)
new_finger = finger[:,taxel] - p(x)

#integral of the signal
det_integral = np.cumsum(new_finger)
integral_original = np.cumsum(finger[:,taxel])

#detrending non lineare del segnale
x = np.arange(len(det_integral))
z = np.polyfit(x, det_integral, 1)
p = np.poly1d(z)
det_integral_det = det_integral - p(x)

#detrending integrale originale
x = np.arange(len(integral_original))
z = np.polyfit(x, integral_original, 1)
p = np.poly1d(z)
integral_det = integral_original - p(x)

new_integral = np.zeros(finger.shape[0])
for e in range(0, len(det_integral_det)):
    if det_integral_det[e] > 0:
        new_integral[e] = det_integral_det[e]

# def butter_highpass(cutoff, fs, order=5):
#     nyquist = 0.5 * fs
#     normal_cutoff = cutoff / nyquist
#     b, a = butter(order, normal_cutoff, btype='high', analog=True)
#     return b, a

# def butter_highpass_filter(data, cutoff, fs, order=5):
#     b, a = butter_highpass(cutoff, fs, order=order)
#     w, h = freqs(b, a)
#     plt.semilogx(w, 20 * np.log10(abs(h)))
#     plt.title('Butterworth filter frequency response')
#     plt.xlabel('Frequency [radians / second]')
#     plt.ylabel('Amplitude [dB]')
#     plt.margins(0, 0.1)
#     plt.grid(which='both', axis='both')
#     plt.axvline(100, color='green') # cutoff frequency
#     plt.show()
#     y = lfilter(b, a, data)
#     return y

#filtro il segnale
fs = 310
cutoff = 50
order = 8

#integral_highpass = butter_highpass_filter(integral_original, cutoff, fs, order)

#subplot per confrontare i segnali
fig, axs = plt.subplots(2, 1)
#axs[0].plot(finger[:,taxel])
axs[0].plot(new_finger)
axs[0].set_title('Derivative signal')
axs[1].plot(det_integral_det)
axs[1].plot(integral_original)
#axs[1].plot(integral_highpass)
#axs[1].plot(p(x))
axs[1].plot(new_integral)
axs[1].set_title('Force signal')
axs[1].grid()
plt.show()

#write a column csv file with the original signal
np.savetxt(dir + "/thumb.csv", finger[:,taxel], delimiter=",")