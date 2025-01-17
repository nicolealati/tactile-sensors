%% ALGORITMO PER L'IDENTIFICAZIONE DEL GRASP DA SEGNALE
clc, clear all
folder = "signals_txt";
test = "cylinder-filter-2";
filter_bool = 1; 
save_fig = 0;

f = funcSelectFilter(filter_bool)

disp(' '), disp(test)

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
finger = sprintf("%sthumb", f)
fullfile(folder, test, sprintf("%s_t%d.txt", finger, 0))
[n_samples, ~] = size(csvread(fullfile(folder, test, sprintf("%s_t%d.txt", finger, 0))));
all_thumb = funcLoadTxt(folder, test, finger, taxel, n_samples);

% finger = "index";
% all_index = funcLoadTxt(folder, test, finger, taxel, n_samples);
% 
% finger = "middle";
% all_middle = funcLoadTxt(folder, test, finger, taxel, n_samples);
% 
% all_signals = [all_thumb; all_index; all_middle];

fig3 = figure(3);
fig3.WindowState = 'maximized';
clf

% Per ogni tassello, plot del segnale filtrato e del valore assoluto
for t = taxel
    subplot(4, 2, t), hold on, grid on
    xlim([0,n_samples])
    ylim([-limit, limit])
    plot(all_thumb(t, :))
    title(sprintf("Taxel %d", t))
end


%%% LOAD SEGNALI e FILTRAGGIO
finger = "thumb";
fullfile(folder, test, sprintf("%s%s_t%d.txt", f, finger, 0))
[n_samples, ~] = size(csvread(fullfile(folder, test, sprintf("%s%s_t%d.txt", f, finger, 0))));
all_thumb_filt = funcLoadTxt(folder, test, finger, taxel, n_samples);

% finger = "index";
% all_index = funcLoadTxt(folder, test, finger, taxel, n_samples);
% 
% finger = "middle";
% all_middle = funcLoadTxt(folder, test, finger, taxel, n_samples);
% 
% all_signals = [all_thumb; all_index; all_middle];


% Per ogni tassello, plot del segnale filtrato e del valore assoluto
for t = taxel
    subplot(4, 2, t), hold on, grid on
    xlim([0,n_samples])
    ylim([-limit, limit])
    plot(all_thumb_filt(t, :))
    title(sprintf("Taxel %d", t))
end