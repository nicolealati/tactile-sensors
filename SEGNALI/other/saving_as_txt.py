## Save .npy records into .txt to be used in Matlab (Barge)

import numpy as np
import os

def save_as_txt(signal_path, taxel, signal_path_save):
    try:
        # Load the .npy file
        signal = np.load(signal_path)
        
        # Check if the taxel index is valid
        if taxel >= len(signal):
            print(f"Taxel index {taxel} is out of bounds for signal with length {len(signal)}")
            return
        
        # Select the specific taxel from the signal
        signal = signal[:,taxel]
        
        # Debug print statements to check what you're saving
        print(f"Signal shape: {signal.shape}")
        print(f"Signal content: {signal}")
        
        # Save the signal as a text file
        np.savetxt(signal_path_save+'\index7.txt', signal, fmt='%i', delimiter=';')
        print('Signal saved as txt file')
    
    except Exception as e:
        print(f"An error occurred: {e}")


def main():
    signal_path = r'C:\Users\nicole\Desktop\2024-2027 - PHD\mano\signals_npy\Cylinder stationary\index.npy'
    signal_path_save = r'C:\Users\nicole\Desktop\2024-2027 - PHD\mano\signals_txt'
    taxel = 7
    
    # Check if the file exists before loading
    if not os.path.exists(signal_path):
        print(f"File not found: {signal_path}")
        return
    
    # Call the function to save the data as text
    save_as_txt(signal_path, taxel,signal_path_save)

if __name__ == "__main__":
    main()
