%% ALGORITMO PER L'IDENTIFICAZIONE DEL GRASP DA SEGNALE
clc, clear all

folder = "signals_txt";
test = "sphere-sliding";
finger = "middle";
disp(' '), disp([test, finger])

save_fig = 0;

% INIZIALIZZAZIONE variabili/parametri per l'identificazione del grasp
taxel = 1:8; % In python, da 0 a 7
fs = 250;

% Filtro passa-banda
% fc = [0.03, 0.3];
fc = [0.02 0.5];
order = 2;
type = 'bandpass';
limit = 30;

threshold_ampiezza = 1;
threshold_samples = 1500;
threshold_ampiezza_2 = threshold_ampiezza*0.4;
tare = 1000;

disp(threshold_samples)
disp(threshold_ampiezza)

%%% LOAD SEGNALI e FILTRAGGIO
[n_samples, ~] = size(csvread(fullfile(folder, test, sprintf("%s_t%d.txt", finger, 0))));

% Pre-allocazione della matrice (8x n_samples) per i segnali
all_signals = zeros(length(taxel), n_samples);

% #RIGA = #TASSELLO (TASSELLO-1 per Python)
for t = taxel

    % Definizione del path in cartella
    txt_path = fullfile(folder, test, sprintf("%s_t%d.txt", finger, t-1));

    % Salvo il segnale in variabile temporanea
    temp_signal = csvread(txt_path);

    % Salvo segnali (grezzo, filtrato, valore assoluto del filtrato
    all_signals(t, :) = round(temp_signal');

end

%%% MEDIA
mean_signal = mean(all_signals, 1);
filt_mean_signal = funcButter(mean_signal, order, fc, fs, type);
abs3_filt_mean_signal = abs(filt_mean_signal).^3;

fig1 = figure(1);
fig1.WindowState = 'maximized';
clf
hold on, grid on

xlim([0,n_samples]), ylim([-0.5*limit, limit])
plot(mean_signal)
plot(filt_mean_signal, 'LineWidth', 1)
plot(abs3_filt_mean_signal, 'LineWidth', 1.5)
yline(threshold_ampiezza,'r-', 'LineWidth', 1.8)
yline(threshold_ampiezza_2,'r-.', 'LineWidth', 1.2)


%%% ALGORITMO CON ABS^3
algoritm_signal = abs3_filt_mean_signal;

grasp = 0;
n_grasp  = 0;

% Iter su ogni valore
for n = 1:n_samples

    current_value = algoritm_signal(n);

    if grasp == 0 %%% NO GRASP

        % Valuto se supera grasp
        if current_value > threshold_ampiezza
            grasp = 1;
            start_grasp = n;

            n_grasp = n_grasp+1 ;

            xline(start_grasp, 'b', 'LineWidth', 1.5)

            % Linea di inizio grasp
            xline(n, 'r-', 'LineWidth', 2)
            text(n, 1.01*limit, sprintf('G%d', n_grasp), 'Color','r','FontWeight','bold', ...
                'FontSize', 9, 'HorizontalAlignment', 'center', 'VerticalAlignment','baseline')
        end

    else %%% GRASP
        % Se il valore è sotto la soglia, inizio a contare e controllo
        % valore
        if current_value < threshold_ampiezza_2
            counter_release = counter_release + 1;

            % FINE GRASP quando counter_release supera il valore di
            % soglia
            if counter_release > threshold_samples % FINE GRASP

                grasp = 0;
                counter_release = 0;

                % Linea di fine grasp
                xline(n, 'r', 'LineWidth', 1.5)
                text(n, 1.01*limit, sprintf('R%d', n_grasp), 'Color','r','FontWeight','bold', ...
                    'FontSize', 9, 'HorizontalAlignment', 'center', 'VerticalAlignment','baseline')

                % Regione di grasp
                xregion(start_grasp, n, 'FaceColor','r', 'FaceAlpha', 0.3)
                text(round(n+start_grasp)/2, 1.01*limit, 'GRASP','Color','r','FontWeight','bold', ...
                    'FontSize', 6, 'HorizontalAlignment', 'center', 'VerticalAlignment','baseline')

                plot(start_grasp:n, algoritm_signal(start_grasp:n), 'Color', 'r', 'LineWidth', 2)

                % Regione della tara
                xline(n-tare, 'k-.', 'LineWidth', 1.5)
                xregion(n-tare, n, 'FaceColor', 'g', 'FaceAlpha', 0.5)
                plot(n-tare:n, algoritm_signal(n-tare:n), 'Color', 'g', 'LineWidth', 2)


            end
        else
            counter_release = 0;
        end
    end

end

text(0, 1.10*limit, sprintf('TEST = %s, FINGER = %s', test, finger), ...
    'FontWeight','bold', FontSize=10)
text(0, 1.06*limit, sprintf('Th Samples = %d, Th Amp = %.2f', threshold_samples, threshold_ampiezza), ...
    'FontWeight','normal', FontSize=8)

if save_fig
    folder_save = sprintf('figures/grasp-identification-mean-abs3/s%d-a%.2f', threshold_samples, threshold_ampiezza);
    funcCreateFolder(folder_save)
    fpath_save = fullfile(folder_save, sprintf('%s-%s-s%d-a%.2f.png', test, finger, threshold_samples, threshold_ampiezza))
    saveas(1, fpath_save);
    % close all
end

disp('END')

