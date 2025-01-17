%% ALGORITMO PER L'IDENTIFICAZIONE DEL GRASP DA SEGNALE
%%%% THRESHOLD TASSELLO PER TASSELLO
clc, clear all

folder = "signals_txt";
test = "cylinder-stationary";
finger = "middle";

filter_bool = 1 ;
f = funcUseFilter(filter_bool);
finger = sprintf("%s%s", f, finger);

disp(' '), disp([test, finger])

save_fig = 0;

threshold_ampiezza_vect = 1.8;
threshold_samples = 1500;
disp(threshold_samples), disp(threshold_ampiezza_vect)

tare = 1000; 

taxel = 1:8; % In python, da 0 a 7
fs = 250;

% Filtro passa-banda
%fc = [0.03, 0.3];
fc = [0.05 5];
order = 2;
type = 'bandpass';

limit = 50;
if filter_bool
    limit = limit/5;
end
  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% LOAD SEGNALI e FILTRAGGIO
[n_samples, ~] = size(csvread(fullfile(folder, test, sprintf("%s-t%d.txt", finger, 0))));

% Pre-allocazione della matrice (8x n_samples) per i segnali
all_signals = zeros(length(taxel), n_samples);
filt_signals = zeros(size(all_signals));
abs_signals =  zeros(size(all_signals));
rms_filtered_signals =  zeros(size(all_signals));

% #RIGA = #TASSELLO (TASSELLO-1 per Python)
for t = taxel

    % Definizione del path in cartella
    txt_path = fullfile(folder, test, sprintf("%s-t%d.txt", finger, t-1));

    % Salvo il segnale in variabile temporanea
    temp_signal = csvread(txt_path);

    % Salvo segnale da txt in matrice
    all_signals(t, :) = temp_signal';

    % Applico filtro (se serve)
    if filter_bool % filtro già applicato onlone
        filt_signals(t, :) = all_signals(t, :);
    else % filtro non applicato online
        filt_signals(t, :) = funcButter(temp_signal', order, fc, fs, type);
    end

    % Calcolo abs e rms
    abs_signals(t, :) = abs(filt_signals(t, :));
    rms_filtered_signals(t, :) = filt_signals(t, :).^2;
end
%%
%%% ALGORITMO
algoritm_signals = abs_signals;
n_figure = 1;

for threshold_ampiezza = threshold_ampiezza_vect

    threshold_ampiezza_2 = threshold_ampiezza/1.5;

    % Inizializzazioe variabili
    grasp = 0;
    counter_release = 0;
    n_grasp = 1;

    n_figure = 1;
    fig1 = figure(n_figure);
    fig1.WindowState = 'maximized';
    clf

    % Per ogni tassello, plot del segnale filtrato e del valore assoluto
    for t = taxel
        subplot(4, 2, t), hold on, grid on
        xlim([0,n_samples])
        ylim([-limit, limit])
        plot(all_signals(t, :))
        plot(filt_signals(t, :), 'LineWidth', 1)
        plot(abs_signals(t, :));
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
                        xregion(n-tare, n, 'FaceColor','g', 'FaceAlpha', 0.2)
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
    text(0, 1.5*limit, sprintf('TEST = %s, FINGER = %s', test, finger),'Color','k', 'FontWeight','bold', FontSize=10)
    text(0, 1.3*limit, sprintf('Th Samples = %d, Th Amp = %.2f', threshold_samples, threshold_ampiezza), FontSize=8)

    %%% SAVE FIGURE
    folder_save = sprintf('figures/fig-grasp-id-taxels/s(%d)a(%.2f)', threshold_samples, threshold_ampiezza);
    name_figure = sprintf('%s-%s-s(%d)a(%.2f).png', test, finger, threshold_samples, threshold_ampiezza);
    
    funcSaveFigure(save_fig, n_figure, folder_save, name_figure, @funcCreateFolder)
    % close all
    
    n_figure = n_figure+1;
end

disp('END')