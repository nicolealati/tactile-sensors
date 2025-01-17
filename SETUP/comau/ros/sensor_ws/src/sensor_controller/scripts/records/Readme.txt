Ogni cartella contiene i valori dei sensori su 9 prese consecutive eseguite su oggetti diversi, considerando o meno lo slittamento 
completo dell'oggetto post presa.

Le prese hanno come differenza la velocità di esecuzione del grasp e il livello di chiusura, che dovrebbe aumentarne la forza.

Vedendo i grafici a gruppi di 3, ogni gruppo ha la stessa intensità di chiusura ma con velocità incrementale, passando al gruppo 
successivo si avrà una forza maggiore con le velocità che si comportano allo stesso modo del precedente.

Ogni file .npy ha una dimensione (n_samples, n_taxels)

cutoff = 30
order = 2

def butter_highpass(cutoff, fs, order=5):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    return b, a

def butter_highpass_filter(data, cutoff, fs, order=5):
    b, a = butter_highpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y
