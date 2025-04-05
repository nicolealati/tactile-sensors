clc, clear all, close all

bool_save = 1
name_figure = '_regression_plot.pdf';

font = 'Helvetica';
size = 10;
size_legend = 8;

line_true = 2;
line_pred = 0.4;

% dimension = [100, 100, 1100, 250];
dimension = [100, 100, 900, 200];

%% load signals HAND
folder = "pressure HAND";
type = 'stft';
time_stft = load(fullfile(folder, strcat(type, ' hand pred_time.txt')));
sign_stft = load(fullfile(folder, strcat(type, ' hand pred_signal.txt')));

type = 'wavelet';
time_wavelet = load(fullfile(folder, strcat(type, ' hand pred_time.txt')));
sign_wavelet = load(fullfile(folder, strcat(type, ' hand pred_signal.txt')));

type = 'raw';
time_raw = load(fullfile(folder, strcat(type, ' hand pred_time.txt')));
sign_raw = load(fullfile(folder, strcat(type, ' hand pred_signal.txt')));

time_true = load(fullfile(folder, strcat(type, ' hand true_time.txt')));
sign_true = load(fullfile(folder, strcat(type, ' hand true_signal.txt')));

type = 'piezo';

time_piezo = load(fullfile(folder, strcat(type, ' hand time.txt')));
sign_piezo = load(fullfile(folder, strcat(type, ' hand signal.txt')));

figure('Position', dimension); % [x, y, larghezza, altezza]
grid on, hold on, box("on")

plot(time_true, sign_true, 'Color',   [0.5, 0, 0.5], 'LineWidth', line_true)
plot(time_raw, sign_raw, 'Color', [0, 0.4, 0.8] , 'LineStyle', '-', 'LineWidth', line_pred)
plot(time_stft, sign_stft, 'Color', [0.8, 0.4, 0] ,'LineStyle', '-' , 'LineWidth', line_pred)
plot(time_wavelet, sign_wavelet, 'Color', [0, 0.5, 0],'LineStyle', '-', 'LineWidth', line_pred)

legend({'f_z', 'raw signal', 'STFT', 'DWT'}, 'Location', 'northeastoutside', 'Orientation', 'vertical', ...
    'FontSize', size_legend, 'FontName', font)
xlabel('Time (s)', 'FontSize', size, 'FontName', font)
ylabel('Force (N)', 'FontSize', size, 'FontName', font)

ylim([-1, 11])
xlim([0, 358])

% title('Vertical force vs Predicted force values', 'FontSize', size+2, 'FontName', font, 'FontWeight', 'BOLD')
% subtitle("With human finger", 'FontSize', size+2, 'FontName', font)
% annotation('textbox', [0.085, 0.739, 0.1, 0.05], 'String', 'a', 'FontWeight', 'bold', ...
%     'EdgeColor', 'none', 'FontSize', size+3, 'FontName', 'Times New Romans', 'HorizontalAlignment', 'left');

name_figure = 'HFR_regression_hand.pdf';

if bool_save
    disp('Saving figure...')
    exportgraphics(gcf, name_figure, 'ContentType', 'vector');
    disp('SAVED')
end

%% load signals ar10
folder = "pressure AR10";
type = 'stft';
time_stft = load(fullfile(folder, strcat(type, ' ar10 pred_time.txt')));
sign_stft = load(fullfile(folder, strcat(type, ' ar10 pred_signal.txt')));

type = 'wavelet';
time_wavelet = load(fullfile(folder, strcat(type, ' ar10 pred_time.txt')));
sign_wavelet = load(fullfile(folder, strcat(type, ' ar10 pred_signal.txt')));

type = 'raw';
time_raw = load(fullfile(folder, strcat(type, ' ar10 pred_time.txt')));
sign_raw = load(fullfile(folder, strcat(type, ' ar10 pred_signal.txt')));

time_true = load(fullfile(folder, strcat(type, ' ar10 true_time.txt')));
sign_true = load(fullfile(folder, strcat(type, ' ar10 true_signal.txt')));

type = 'piezo';
time_piezo = load(fullfile(folder, strcat(type, ' ar10 time.txt')));
sign_piezo = load(fullfile(folder, strcat(type, ' ar10 signal.txt')));

figure('Position', dimension); % [x, y, larghezza, altezza]
grid on, hold on, box("on")

plot(time_true, sign_true, 'Color',   [0, 0, 0.5], 'LineWidth', line_true)
plot(time_raw, sign_raw, 'Color', [0, 0.4, 0.8] , 'LineStyle', '-', 'LineWidth', line_pred)
plot(time_stft, sign_stft, 'Color', [0.8, 0.4, 0] ,'LineStyle', '-' , 'LineWidth', line_pred)
plot(time_wavelet, sign_wavelet, 'Color', [0, 0.5, 0],'LineStyle', '-', 'LineWidth', line_pred)

legend({'f_z', 'raw signal', 'STFT', 'DWT'}, 'Location', 'northeastoutside', 'Orientation', 'vertical', ...
    'FontSize', size_legend, 'FontName', font)
xlabel('Time (s)', 'FontSize', size, 'FontName', font)
ylabel('Force (N)', 'FontSize', size, 'FontName', font)

ylim([-1, 7])
xlim([0,274])


% title('Vertical force vs Predicted force values', 'FontSize', size+2, 'FontName', font, 'FontWeight', 'BOLD')
% subtitle("With robotic finger", 'FontSize', size+3, 'FontName', font)
% annotation('textbox', [0.085, 0.25, 0.1, 0.05], 'String', 'b', 'FontWeight', 'bold', ...
%     'EdgeColor', 'none', 'FontSize', size+2, 'FontName', 'Times New Romans', 'HorizontalAlignment', 'left');


name_figure = 'HFR_regression_ar10.pdf';

if bool_save
    disp('Saving figure...')
    exportgraphics(gcf, name_figure, 'ContentType', 'vector');
    disp('SAVED')
end
disp('END')





