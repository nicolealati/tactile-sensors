function [grasp_region, start_grasp, stop_grasp] = funcThresholdAlgorithm(filt_all, abs_all, funcShift, ...
    taxels, n_fingers, n_tare, ...
    th_start_grasp, th_stop_grasp, th_samples)
%%%%%%%%%%%%%%%%%%%%%%%% ALGORITMO (TASSELLO x TASSELLO)

%%% PRE ALLOCAZIONI 
grasp_region = NaN(size(abs_all));  %% [24xN] 1 se sono in grasp, 0 se non sono in grasp
grasp_region(:,1) = 0;

counter_release = NaN(size(abs_all,1), 1); %% [24x1] conto il grasp per ogni tassello 
counter_grasp = zeros(size(abs_all,1), 1); %% [24x1] conto i grasp identificati per ogni tassello 

start_grasp = NaN(size(abs_all)); %% [24xN] 1 al campione di inizio grasp, altrimenti Nan
stop_grasp = NaN(size(abs_all)); %% [24xN] 1 al campione di fine grasp, altrimenti Nan
% taxel_grasp = NaN(size(abs_all)); %% [24xn] 

%%% Matrice FIFO (ad ogni ciclo, shift a sinistra e inserisco il valore corrente)
tare_filt= NaN(size(filt_all, 1), n_tare); 
tare_filt(:, end) = filt_all(:, 1);
tare_abs= zeros(size(abs_all, 1), n_tare); 
tare_abs(:, end) = abs_all(:, 1); 

for step = 2:size(abs_all, 2)
    current_abs = abs_all(:, step);  %% estraggo valori al campione corrente
    
    tare_filt = funcShift(tare_filt, filt_all(:, step));  %% shift filtrato
    tare_abs  = funcShift(tare_abs,  abs_all(:, step));   %% shoft valore assoluto

    for t = taxels
        for n_f = 1:n_fingers
            line = t+8*(n_f-1); %% estraggo linea corrispondente al tassello della mano corrente 

            %%%%%% NO GRASP al campione precedente
            if grasp_region(line, step-1) == 0  
                
                %%%% START GRASP
                if current_abs(line)>th_start_grasp 
                    grasp_region(line, step) = 1;                       %% salvo "1" nel campione corrente alla linea corrispondente 
                    % taxel_grasp(line, n) = t;                         %% salvo il tassello di inizio grasp alla linea corrispondente
                    start_grasp(line, step) = step;                     %% salvo il campione di fine grasp alla linea corrispondente
                    % counter_grasp(line) = counter_grasp(line)+1;        %% counto grasp alla linea corrispondente
                                 
                %%%% NO GRASP
                else 
                    grasp_region(line, step) = 0;  %% salvo "0" nel campione corrente alla linea corrispondente
                end

            %%%%%% GRASP al campione precedente
            else 
                %%%% POTENZIALE FINE GRASP
                if current_abs(line)<th_stop_grasp 

                    counter_release(line) = counter_release(line)+1; %% inizio a contare i campioni sotto soglia
                    
                    %%% FINE GRASP
                     if counter_release(line)>th_samples    %% counter al valoredi soglia
                        grasp_region(line,step-th_samples:step) = 0;            %% salvo "0" nei 1500 campione prima del corente alla linea corrispondente 
                        counter_release(line) = 0;                              %% reset counter alla linea corrispondente 
                        stop_grasp(line, step-th_samples) = step-th_samples;                     %% salvo il campione-1500 di fine grasp alla linea corrispondente
                    
                    %%% ANCORA IN GRASP
                    else
                        grasp_region(line, step) = 1;  %% salvo "1" nel campione corrente alla linea corrispondente 
                    end
                
                %%%% ANCORA IN GRASP
                else 
                    counter_release(line) = 0;      %% reset counter perchè ancora in grasp 
                    grasp_region(line, step) = 1;      %% salvo "1" nel campione corrente alla linea corrispondente 
                end
            end
        end
    end
end

disp("Algorithm ended")
end

