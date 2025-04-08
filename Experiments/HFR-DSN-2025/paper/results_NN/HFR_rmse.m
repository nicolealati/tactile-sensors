clc, clear all, close all

bool_save = 0
name_figure = 'HFR_rmse.pdf';

% colors = [0.5, 0, 0.5;  % Viola chiaro
%                    0, 0, 0.5];   % Azzurro (#A0C4FF)
colors = [0.8667 0.6784 0.8745;  % Viola chiaro
    0.6275 0.7686 1.0000]; % Azzurro (#A0C4FF)

font = 'Helvetica';
size = 10;
size_legend = 8;

figure('Position', [100, 100, 400, 150]); % [x, y, larghezza, altezza]
%figure; set(gcf, 'Position', get(0, 'ScreenSize'));

%% Load signals
folder = "pressure HAND";

pred_hand_raw = load(fullfile(folder, strcat('raw', ' hand pred_signal.txt')));
true_hand_raw = load(fullfile(folder, strcat('raw', ' hand true_signal.txt')));
xq = linspace(1, length(pred_hand_raw), floor(length(pred_hand_raw)*1));
pred_hand_raw_interp = interp1(1:length(pred_hand_raw), pred_hand_raw, xq, 'linear');
true_hand_raw_interp = interp1(1:length(true_hand_raw), true_hand_raw, xq, 'linear');
rmse_hand_raw = rmse(pred_hand_raw_interp, true_hand_raw_interp);

pred_hand_stft = load(fullfile(folder, strcat('stft', ' hand pred_signal.txt')));
true_hand_stft = load(fullfile(folder, strcat('stft', ' hand true_signal.txt')));
xq = linspace(1, length(pred_hand_stft), floor(length(pred_hand_stft)*1));
pred_hand_stft_interp = interp1(1:length(pred_hand_stft), pred_hand_stft, xq, 'linear');
true_hand_stft_interp = interp1(1:length(true_hand_stft), true_hand_stft, xq, 'linear');
rmse_hand_stft = rmse(pred_hand_stft_interp, true_hand_stft_interp);

pred_hand_wavelet = load(fullfile(folder, strcat('wavelet', ' hand pred_signal.txt')));
true_hand_wavelet = load(fullfile(folder, strcat('wavelet', ' hand true_signal.txt')));
xq = linspace(1, length(pred_hand_wavelet), floor(length(pred_hand_wavelet)*1));
pred_hand_wavelet_interp = interp1(1:length(pred_hand_wavelet), pred_hand_wavelet, xq, 'linear');
true_hand_wavelet_interp = interp1(1:length(true_hand_wavelet), true_hand_wavelet, xq, 'linear');
rmse_hand_wavelet = rmse(pred_hand_wavelet_interp, true_hand_wavelet_interp);

folder = "pressure AR10";

pred_ar10_raw = load(fullfile(folder, strcat('raw', ' ar10 pred_signal.txt')));
true_ar10_raw = load(fullfile(folder, strcat('raw', ' ar10 true_signal.txt')));
xq = linspace(1, length(pred_ar10_raw), floor(length(pred_ar10_raw)*1));
pred_ar10_raw_interp = interp1(1:length(pred_ar10_raw), pred_ar10_raw, xq, 'linear');
true_ar10_raw_interp = interp1(1:length(true_ar10_raw), true_ar10_raw, xq, 'linear');
rmse_ar10_raw = rmse(pred_ar10_raw_interp, true_ar10_raw_interp);

pred_ar10_stft = load(fullfile(folder, strcat('stft', ' ar10 pred_signal.txt')));
true_ar10_stft = load(fullfile(folder, strcat('stft', ' ar10 true_signal.txt')));
xq = linspace(1, length(pred_ar10_stft), floor(length(pred_ar10_stft)*1));
pred_ar10_stft_interp = interp1(1:length(pred_ar10_stft), pred_ar10_stft, xq, 'linear');
true_ar10_stft_interp = interp1(1:length(true_ar10_stft), true_ar10_stft, xq, 'linear');
rmse_ar10_stft = rmse(pred_ar10_stft_interp, true_ar10_stft_interp);

pred_ar10_wavelet = load(fullfile(folder, strcat('wavelet', ' ar10 pred_signal.txt')));
true_ar10_wavelet = load(fullfile(folder, strcat('wavelet', ' ar10 true_signal.txt')));
xq = linspace(1, length(pred_ar10_wavelet), floor(length(pred_ar10_wavelet)*1));
pred_ar10_wavelet_interp = interp1(1:length(pred_ar10_wavelet), pred_ar10_wavelet, xq, 'linear');
true_ar10_wavelet_interp = interp1(1:length(true_ar10_wavelet), true_ar10_wavelet, xq, 'linear');
rmse_ar10_wavelet = rmse(pred_ar10_wavelet_interp, true_ar10_wavelet_interp);

%%   GROUP 

with_ar10 = [rmse_ar10_raw rmse_ar10_stft, rmse_ar10_wavelet] 
without_ar10 = [rmse_hand_raw, rmse_hand_stft, rmse_hand_wavelet]

bar_all = bar([without_ar10; with_ar10]', 'grouped');
for k = 1:length(bar_all)
    bar_all(k).FaceColor = colors(k, :);
end
groups = {'raw signals', 'STFT', 'DWT'};
set(gca, 'XTickLabel', groups, 'FontSize', size-2, 'FontName', font)

legend({'human finger', 'robotic finger'}, 'Location', 'northeastoutside', 'FontSize', size_legend, 'FontName', font)
%xlabel('Features', 'FontSize', size, 'FontName', font)
ylabel('RMSE', 'FontSize', size, 'FontName', font)
ylim([0, 2])

% title('pREESSURE TASKS', 'FontSize', size, 'FontWeight', 'bold', 'FontName', font)
%subtitle("SVM for contact", 'FontSize', size, 'FontName', font)
% annotation('textbox', [0.055, 0.74, 0.1, 0.05], 'String', 'a', ...
%     'EdgeColor', 'none', 'FontSize', size-1, 'FontName', 'Times New Romans', 'HorizontalAlignment', 'left');

for k = 1:length(bar_all)  
    xtips = bar_all(k).XEndPoints; 
    ytips = bar_all(k).YEndPoints; 
    labels = string(round(bar_all(k).YData, 2)); % Arrotonda al terzo decimale
    text(xtips, ytips + 0.02, labels, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
        'FontSize', size-2, 'FontName', 'Helvetica', 'FontWeight', 'normal');
end
box("off")

if bool_save
    disp('Saving figure...')
    exportgraphics(gcf, name_figure, 'ContentType', 'vector');
    disp('SAVED')
end
disp('END')
