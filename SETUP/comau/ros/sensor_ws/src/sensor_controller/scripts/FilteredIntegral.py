#!/usr/bin/env python3

import numpy as np
import rospy
from sensor_controller.msg import Forcesensor, Piezosensor
from sensor_controller.srv import Integral, IntegralResponse
from sensor_controller.srv import Tare, TareRequest
from scipy.signal import butter, lfilter

class IntegralSensor:
    def __init__(self):
        self.start_integral = False
        rospy.init_node('integral_controller')
        self.IntegralService = rospy.Service('/integral', Integral, self.integral)
        self.TareClient = rospy.ServiceProxy('/tare', Tare)
        rospy.wait_for_service('/tare')
        self.Sub = rospy.Subscriber('/piezosensor', Piezosensor, self.integral_sensor_callback)
        self.Pub = rospy.Publisher('/integral_sensor', Forcesensor, queue_size=10)

        self.window_length = 2000
        self.window = np.zeros((8, 5, self.window_length))
        self.filtered_window = np.zeros((8, 5, self.window_length))  # Preallocata per evitare allocazioni continue
        self.matrix_signal = np.zeros((8, 5))  # Preallocata per evitare allocazioni continue
        self.integral_signal = np.zeros((8, 5))
        self.window_index = 0
        self.fs = 310
        self.lowcut = 5
        self.highcut = 100
        self.order = 3
        self.n_compact = 4
        self.n_counter = 0

    def integral_sensor_callback(self, data):
        if self.start_integral:
            start_time = rospy.Time.now()
            # Aggiornamento matrix_signal in modo vettorizzato
            self.matrix_signal[:, 0] = data.thumb
            self.matrix_signal[:, 1] = data.index
            self.matrix_signal[:, 2] = data.middle
            self.matrix_signal[:, 3] = data.ring
            self.matrix_signal[:, 4] = data.little

            if self.window_index < self.window_length:
                self.window[:, :, self.window_index] = self.matrix_signal
                self.window_index += 1
            else:
                self.window[:, :, :-1] = self.window[:, :, 1:]  # Shift senza copia
                self.window[:, :, -1] = self.matrix_signal  # Aggiunge nuovo segnale
                self.n_counter += 1

                if self.n_counter == self.n_compact:
                    # Filtro in modo vettorizzato
                    #self.filtered_window = self.fft_filter(self.window, self.fs)
                    self.filtered_window = self.window - np.mean(self.window, axis=2)[:, :, np.newaxis]

                    # Calcolo dell'integrale direttamente in vettorizzazione sugli ultimi n_compact segnali
                    self.integral_signal += np.sum(self.filtered_window[:,:,-4:], axis=2)/self.fs

                    # Pubblicazione del messaggio
                    msg_new = Forcesensor()
                    msg_new.thumb = list(self.integral_signal[:, 0])
                    msg_new.index = list(self.integral_signal[:, 1])
                    msg_new.middle = list(self.integral_signal[:, 2])
                    msg_new.ring = list(self.integral_signal[:, 3])
                    msg_new.little = list(self.integral_signal[:, 4])
                    self.Pub.publish(msg_new)
                    self.n_counter = 0

    def fft_filter(self, window, fs):
        # Vettorizzazione del filtro
        filtered_window = np.apply_along_axis(self.butter_bandpass_filter, 2, window, self.lowcut, self.highcut, fs)
        return filtered_window

    def butter_bandpass(self, lowcut, highcut, fs):
        order = self.order
        nyq = 0.5 * fs
        low = lowcut / nyq
        high = highcut / nyq
        b, a = butter(order, [low, high], btype='band')
        return b, a

    def butter_bandpass_filter(self, data, lowcut, highcut, fs, order=1):
        b, a = self.butter_bandpass(lowcut, highcut, fs)
        y = lfilter(b, a, data)
        return y

    def integral(self, req):
        if req.start:
            self.start_integral = False
            self.TareClient(TareRequest(True))
            self.window.fill(0)  # Reset finestra senza riassegnare memoria
            self.integral_signal.fill(0)  # Reset integrale
            self.window_index = 0
            rospy.sleep(5)  # Attesa per stabilizzazione
            self.start_integral = True
            print("Integral started")
        else:
            self.start_integral = False
            print("Integral stopped")
        return IntegralResponse(True)

if __name__ == '__main__':
    integral_sensor = IntegralSensor()
    rospy.spin()
