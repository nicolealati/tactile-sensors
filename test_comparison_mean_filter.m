clc, clear all

folder = "signals_txt";
test = "cylinder-stationary";
disp(' '), disp([test])

save_fig = 1; 

% INIZIALIZZAZIONE variabili/parametri per l'identificazione del grasp
taxel = 1:8; % In python, da 0 a 7
fs = 250;

% Filtro passa-banda
% fc = [0.03, 0.3];
fc = [0.03 0.5];
order = 2;
type = 'bandpass';
limit = 5;

finger = "thumb";
[n_samples, ~] = size(csvread(fullfile(folder, test, sprintf("%s_t%d.txt", finger, 0))));
all_thumb = funcLoadTxt(folder, test, finger, taxel, n_samples);
finger = "index";
all_index = funcLoadTxt(folder, test, finger, taxel, n_samples);
finger = "middle";
all_middle = funcLoadTxt(folder, test, finger, taxel, n_samples);

all_signals = [all_thumb; all_index; all_middle];

fig1 = figure(1);
fig1.WindowState = 'maximized';
clf

%%% TEST MEDIA - FILTRO 
mean_signal = mean(all_signals, 1);
filt_mean_signal = funcButter(mean_signal, order, fc, fs, type);
abs_filt_mean_signal = abs(filt_mean_signal).^3;

subplot(311), hold on, grid on
xlim([0,n_samples]), ylim([-0.5*limit, limit])
plot(filt_mean_signal, 'LineWidth',  1.2, 'Color', 'k')
plot(abs_filt_mean_signal, 'LineWidth', 2, 'Color', 'b')

legend('filt-mean', 'abs^3')
title('MEDIA ---> FILTRO ')

%%% TEST FILTRO - MEDIA 

filtered_signal = zeros(24, n_samples); 
for line = 1:24
    filt_mean_signal(line, :) = funcButter(all_signals(line, :), order, fc, fs, type);
end
mean_filt_signal = mean(filt_mean_signal, 1);
abs_mean_filt_signal = abs(mean_filt_signal).^3;

subplot(312), hold on, grid on
xlim([0,n_samples]), ylim([-0.5*limit, limit])
plot(mean_filt_signal, 'LineWidth', 1.2, 'Color', 'k')
plot(abs_mean_filt_signal, 'LineWidth', 2, 'Color', 'r')

legend('mean-filt', 'abs^3')
title('FILTO ---> MEDIA ')

%%% PARAGONE

subplot(313), hold on, grid on
xlim([0,n_samples]), ylim([-0.5*limit, limit])
plot(abs_filt_mean_signal, 'LineWidth', 2, 'Color', 'b')
plot(abs_mean_filt_signal+0.3, 'LineWidth', 2, 'Color', 'r')

legend('filt-mean', 'mean-filt')
title('PARAGONE')


if save_fig
    folder_save = 'figures/comparison_order_mean_filter';
    funcCreateFolder(folder_save)
    fpath_save = fullfile(folder_save, sprintf('%s.png', test))
    saveas(1, fpath_save);
    % close all
end



disp('END')