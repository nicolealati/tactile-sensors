import os
import numpy as np
import pandas as pd

if os.name == 'nt':
    os.system('cls')

test = 'rolling'

while True:

    try:
        dir = rf"D:\GitHub\tactile-sensors\SETUP\sensore-forza\sequences\{test}"
        sequence_name = f"{test}_sequence.npy"
        xlsx_name = f"pattern_{test}.xlsx"

        df_pattern = pd.read_excel(xlsx_name, sheet_name=test)    
        for col in df_pattern.columns:
            df_pattern[col] = None
        with pd.ExcelWriter(xlsx_name, mode = 'a', engine='openpyxl', if_sheet_exists='overlay') as workbook:    
            df_pattern.to_excel(workbook, sheet_name=test, index=False)

        sequence = np.load(f"{dir}\{sequence_name}")
        time_intervals = [0, 10]
        time_steps = [0, 10]

        for pair in sequence:
            time_intervals.append(pair[1])
            prev_step = time_steps.append(np.sum(time_intervals))

        df_pattern['time_intervals'] = time_intervals + [5]
        df_pattern['time_steps'] = time_steps + [time_steps[-1]+5]

        x_pos = [0, 0]
        y_pos = [0, 0]
        angle_pos_1 = [0, 0]
        angle_pos_2 = [0, 0]

        trigger_sequence =  [int(s[0]) for s in sequence]
        df_pattern['trigger_index'] = [0, 0] + trigger_sequence + [0]

        for t in trigger_sequence: 
            if t == 0: # stop
                x_pos.append(x_pos[-1])
                y_pos.append(y_pos[-1])
                angle_pos_1.append(angle_pos_1[-1])
                angle_pos_2.append(angle_pos_2[-1])

            elif t == 1: # right 
                x_pos.append(x_pos[-1]+1)
                y_pos.append(y_pos[-1])
                angle_pos_1.append(angle_pos_1[-1])
                angle_pos_2.append(angle_pos_2[-1])

            elif t == 2: # left
                x_pos.append(x_pos[-1]-1)
                y_pos.append(y_pos[-1])
                angle_pos_1.append(angle_pos_1[-1])
                angle_pos_2.append(angle_pos_2[-1])

            elif t == 3: # up
                x_pos.append(x_pos[-1])
                y_pos.append(y_pos[-1]+1)
                angle_pos_1.append(angle_pos_1[-1])
                angle_pos_2.append(angle_pos_2[-1])
            
            elif t == 4: # down
                x_pos.append(x_pos[-1])
                y_pos.append(y_pos[-1]-1)
                angle_pos_1.append(angle_pos_1[-1])
                angle_pos_2.append(angle_pos_2[-1])

            elif t == 5: # clockwise
                x_pos.append(x_pos[-1])
                y_pos.append(y_pos[-1])
                angle_pos_1.append(angle_pos_1[-1]+45)
                angle_pos_2.append(angle_pos_2[-1])

            elif t == 6: # clockwise
                x_pos.append(x_pos[-1])
                y_pos.append(y_pos[-1])
                angle_pos_1.append(angle_pos_1[-1]-45)
                angle_pos_2.append(angle_pos_2[-1])

            elif t == 7: # clockwise
                x_pos.append(x_pos[-1])
                y_pos.append(y_pos[-1])
                angle_pos_1.append(angle_pos_1[-1])
                angle_pos_2.append(angle_pos_2[-1]+45)

            elif t == 8: # clockwise
                x_pos.append(x_pos[-1])
                y_pos.append(y_pos[-1])
                angle_pos_1.append(angle_pos_1[-1])
                angle_pos_2.append(angle_pos_2[-1]-45)

        df_pattern['x_position'] = x_pos + [x_pos[-1]]
        df_pattern['y_position'] = y_pos + [y_pos[-1]]
        df_pattern['angle_rotation_1'] = angle_pos_1 + [angle_pos_1[-1]]
        df_pattern['angle_rotation_2'] = angle_pos_2 + [angle_pos_1[-1]]

        print(df_pattern)

        for col in df_pattern.columns:
            print(f"Colonna {col} - ", len(df_pattern[col]))

        with pd.ExcelWriter(xlsx_name, mode = 'a', engine='openpyxl', if_sheet_exists='overlay') as workbook:    
            df_pattern.to_excel(workbook, sheet_name=test, index=False)
        
        break

    except Exception as e:
        # Stampa l'errore e continua
        print(f"Errore durante l'iterazione: {e}")
        continue
