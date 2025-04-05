clc, clear all, close all

bool_save = 1
name_figure = 'HFR_rmse_heatmap.pdf';

font = 'Helvetica';
size = 10;

dimension = [100, 100, 300, 100]; 
map_type = ['sky']; 

figure('Position', dimension); % [x, y, larghezza, altezza]

% Load signals
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

% data = [rmse_hand_raw, rmse_ar10_raw;
%         rmse_hand_stft, rmse_ar10_stft;
%         rmse_hand_wavelet, rmse_ar10_wavelet]';
% 
% features = {'raw signals', 'STFT', 'DWT marg.'};
% groups = {'Human finger', 'Robotic finger'};
% 
% h = heatmap(features, groups, data);
% % h.Title = 'RMSE for different features';
% % h.XLabel = 'Group';
% % h.YLabel = 'Feature type';
% h.FontName = font;
% h.FontSize = size;
% h.CellLabelFormat = '%.2f';
% h.CellLabelColor = 'black';
% 
% colormap(h, map_type);

data = [rmse_hand_raw, rmse_hand_stft, rmse_hand_wavelet; rmse_ar10_raw, rmse_ar10_stft, rmse_ar10_wavelet]
    
features = {'raw s.', 'STFT', 'DWT marg.'};
groups = {'Human f.', 'Robotic f.'};

h = heatmap(groups, features, data');
h.FontName = font;
h.FontSize = size;
h.CellLabelFormat = '%.2f';
h.CellLabelColor = 'black';

colormap(h, map_type);
caxis([0 2]);

% Save 
if bool_save
    disp('Saving figure...')
    exportgraphics(gcf, name_figure, 'ContentType', 'vector');
    disp('SAVED')
end
disp('END')