## Save .npy records into .txt to be used in Matlab
import os
import numpy as np

def CleanTerminal():
    if os.name == 'nt':
        _ = os.system('cls')

def CreateFolder(path):
    if not os.path.exists(path):
        os.makedirs(path)
        return

def SaveNewFormat(load_file_path, taxel, save_file_path):
    signal = np.load(load_file_path)
    np.savetxt(save_file_path, signal[:,taxel], fmt='%f', delimiter=';')

def main():

    CleanTerminal()

    project_path = r'D:\GitHub\tactile-sensors\signals'
    project_save = r'D:\GitHub\tactile-sensors\signals'
    
    original_format = "npy"
    save_format = "txt"

    ### NO FILTRO 
    folder_list = ["cube-sliding", "cube-stationary", 
                   "cylinder-sliding", "cylinder-stationary", "cylinder-filter-1", "cylinder-filter-2",
                   "noise-grasp", "noise-grasp-stationary", "noise-stationary", 
                   "push-to-thumb", 
                   "sphere-sliding", "sphere-stationary",
                   ]
    
    finger_list = ["thumb", "index", "middle", "ring", "little", 
                    "filtered-thumb", "filtered-index", "filtered-middle", "filtered-ring", "filtered-little" ]


    taxel_list = list(range(8))
    # taxel_list = [str(taxel) for taxel in list(range(8))]

    for folder in folder_list:
        for finger in finger_list:
            for taxel in taxel_list:

                save_folder_path = rf"{project_save}_{save_format}\{folder}" 
                CreateFolder(save_folder_path)

                load_file_path = rf"{project_path}_{original_format}\{folder}\{finger}.{original_format}" 
                save_file_path = rf"{save_folder_path}\{finger}-t{taxel}.{save_format}"

                print(load_file_path)
                print(save_file_path)
                print('----')

                if os.path.exists(load_file_path):
                    SaveNewFormat(load_file_path, taxel, save_file_path)
                    print("SAVED")

if __name__ == "__main__":
    main()


