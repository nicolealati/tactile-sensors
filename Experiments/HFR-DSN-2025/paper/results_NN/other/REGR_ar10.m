clc, clear all, close all

bool_save = 1
name_figure = 'REGR_AR10-plot-results.pdf'; 

%% load signals
folder = "pressure AR10"; 
type = 'stft'; 
time_stft = load(fullfile(folder, strcat(type, ' pred_time.txt')));  
sign_stft = load(fullfile(folder, strcat(type, ' pred_signal.txt')));  

type = 'wavelet'; 
time_wavelet = load(fullfile(folder, strcat(type, ' pred_time.txt')));  
sign_wavelet = load(fullfile(folder, strcat(type, ' pred_signal.txt')));  
 
% type = 'raw'; 
% time_raw = load(fullfile(folder, strcat(type, ' pred_time.txt')));  
% sign_raw = load(fullfile(folder, strcat(type, ' pred_signal.txt')));  

time_true = load(fullfile(folder, strcat(type, ' true_time.txt')));  
sign_true = load(fullfile(folder, strcat(type, ' true_signal.txt')));  

type = 'piezo'; 
time_piezo = load(fullfile(folder, strcat(type, ' time.txt')));  
sign_piezo = load(fullfile(folder, strcat(type, ' signal.txt')));  

%% plot figure

font = 'Helvetica';
size = 10; 
size_legend = 8;

close all
figure; set(gcf, 'Position', get(0, 'ScreenSize'));

subplot(2,1,1), grid on
plot(time_piezo, mean(sign_piezo,2), 'Color', [0.3, 0.3, 0.3], 'LineWidth', 1)
title('Tactile signals from TFS', 'FontSize', size, 'FontName', font, 'FontWeight', 'normal')
legend({'mean 8 piezo '}, 'Location', 'northeastoutside', 'FontSize', size_legend, 'FontName', font)

line_true = 1.3;
line_pred = 0.7;

subplot(2,1,2), grid on, hold on
plot(time_true, sign_true, 'Color', [0, 0, 0.5], 'LineWidth', line_true)
plot(time_stft, sign_stft+0.3*sign_wavelet, 'Color', [0, 0.4, 0.8] , 'LineWidth', line_pred)
plot(time_stft, sign_stft, 'Color', [0.8, 0.4, 0] , 'LineWidth', line_pred)
plot(time_wavelet, sign_wavelet, 'Color', [0, 0.5, 0], 'LineWidth', line_pred)
legend({'True', 'RAW', 'STFT', 'DWT'}, 'Location', 'northeastoutside', 'FontSize', size_legend, 'FontName', font)
title('Vertical force vs Predicted force values', 'FontSize', size, 'FontName', font, 'FontWeight', 'normal')
sgtitle('Regression with AR10', 'FontSize', size+4, 'FontName', font, 'FontWeight', 'bold')

%% save figure
if bool_save
    disp('Saving figure...')
    exportgraphics(gcf, name_figure, 'ContentType', 'vector');
    disp('SAVED')
end
disp('END')

close
