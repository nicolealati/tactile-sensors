%% ALGORITMO PER L'IDENTIFICAZIONE DEL GRASP DA SEGNALE
clc, clear all

folder = "signals_txt";
test = "cylinder-stationary";
finger = "index";
disp(' '), disp([test, finger])

save_fig = 0;

% INIZIALIZZAZIONE variabili/parametri per l'identificazione del grasp
taxel = 1:8; % In python, da 0 a 7
fs = 250;

% Filtro passa-banda
fc = [0.03, 0.3];
% fc = [0.05 5]; 
order = 2;
type = 'bandpass';
limit = 50;

threshold_ampiezza_vect = 1.8;
threshold_samples = 1500;

disp(threshold_samples)
disp(threshold_ampiezza_vect)

%%% LOAD SEGNALI e FILTRAGGIO
[n_samples, ~] = size(csvread(fullfile(folder, test, sprintf("%s_t%d.txt", finger, 0))));

% Pre-allocazione della matrice (8x n_samples) per i segnali
all_signals = zeros(length(taxel), n_samples);
filtered_signals = zeros(size(all_signals));
abs_filtered_signals =  zeros(size(all_signals));
rms_filtered_signals =  zeros(size(all_signals));

% #RIGA = #TASSELLO (TASSELLO-1 per Python)
for t = taxel

    % Definizione del path in cartella
    txt_path = fullfile(folder, test, sprintf("%s_t%d.txt", finger, t-1));

    % Salvo il segnale in variabile temporanea
    temp_signal = csvread(txt_path);

    % Salvo segnali (grezzo, filtrato, valore assoluto del filtrato
    all_signals(t, :) = temp_signal';
    filtered_signals(t, :) = funcButter(temp_signal', order, fc, fs, type);
    abs_filtered_signals(t, :) = abs(filtered_signals(t, :));
    rms_filtered_signals(t, :) = filtered_signals(t, :).^2;
end

%%% ALGORITMO
algoritm_signals = abs_filtered_signals;

for threshold_ampiezza = threshold_ampiezza_vect

    threshold_ampiezza_2 = threshold_ampiezza/1.5; 

    % Inizializzazioe variabili
    grasp = 0;
    counter_release = 0;
    n_grasp = 1;

    fig3 = figure(3);
    fig3.WindowState = 'maximized';
    clf

    % Per ogni tassello, plot del segnale filtrato e del valore assoluto
    for t = taxel
        subplot(4, 2, t), hold on, grid on
        xlim([0,n_samples])
        ylim([-limit, limit])
        plot(all_signals(t, :))
        plot(filtered_signals(t, :), 'LineWidth', 1)
        plot(abs_filtered_signals(t, :));
        % plot(rms_filtered_signals(t, :), 'y')
        yline(threshold_ampiezza,'k--', 'LineWidth', 1.5)
        yline(threshold_ampiezza_2,'k-.', 'LineWidth', 1)
    end
   
    % Iter su ogni valore
    for n = 1:n_samples
        current_value = algoritm_signals(:, n); % vettore colonna 8x1

        if grasp == 0 %%% NO GRASP
            % Valuto per ogni tassello, valuto se supera grasp
            for t = taxel
                if current_value(t, 1) > threshold_ampiezza
                    grasp = 1;
                    taxel_grasp = t; %%%% PRENDE ULTIMO TASSELLO? CONCATENO?
                    start_grasp = n;

                    subplot(4,2,taxel_grasp)
                    xline(start_grasp, 'b', 'LineWidth', 1.5)
                end
            end
            % taxel_grasp ha il valore dell'ultimo tassello con valore
            % sopra soglia


        else %%% GRASP
            % Se il valore è sotto la soglia, inizio a contare e controllo
            % valore
            if current_value(taxel_grasp) < threshold_ampiezza_2
                counter_release = counter_release + 1;
                % FINE GRASP quando counter_release supera il valore di
                % soglia
                if counter_release > threshold_samples % FINE GRASP

                    grasp = 0;
                    counter_release = 0;

                    for t = taxel
                        subplot(4,2,t)
                        ylim([-limit, limit])
                        text(round(n_samples/2), limit*1.2, sprintf('Taxel %d', t), 'FontWeight', 'normal', FontSize=9, ...
                            HorizontalAlignment='center')
                        % Linea di fine grasp
                        xline(n, 'k', 'LineWidth', 1)
                        text(n+200, -0.9*limit, sprintf('%d', n_grasp),'Color','k','FontWeight','bold', FontSize=8)
                        % Regione di grasp
                        xregion(start_grasp, n, 'FaceColor','b', 'FaceAlpha', 0.1)
                        text(round(n+start_grasp)/2, 1.001*limit, 'GRASP','Color','b','FontWeight','normal', FontSize=6, ...
                            HorizontalAlignment='center', VerticalAlignment='baseline')
                        % Regione della tara
                        xregion(n-1000, n, 'FaceColor','g', 'FaceAlpha', 0.2)
                    end

                    % Plot fine grasp nel tasssello identificato e il
                    % numero del grasp
                    subplot(4,2,taxel_grasp)
                    xline(n, 'r', 'LineWidth', 1.5)
                    text(n+200, -0.9*limit, sprintf('%d', n_grasp),'Color','r', 'FontWeight','bold', FontSize=8)
                    n_grasp = n_grasp+1 ;

                end
            else %% serve?
                counter_release = 0;
            end

            %%%%%
        end
    end

    subplot(421)
    text(0, 90, sprintf('TEST = %s, FINGER = %s', test, finger),'Color','k', 'FontWeight','bold', FontSize=10)
    text(0, 75, sprintf('Th Samples = %d, Th Amp = %d', threshold_samples, threshold_ampiezza), FontSize=8)
    
    if save_fig
        folder_save = sprintf('figures/grasp-identification/s%d-a%d', threshold_samples, threshold_ampiezza);
        funcCreateFolder(folder_save)
        fpath_save = fullfile(folder_save, sprintf('%s-%s-s%d-a%d.png', test, finger, threshold_samples, threshold_ampiezza));
        saveas(3, fpath_save);
        close all
    end
end

disp('END')