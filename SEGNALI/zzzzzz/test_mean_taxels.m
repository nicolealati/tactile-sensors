% TEST  MEDIA TASSELLI
clc, clear all

folder = "signals_txt";
test = "cylinder-stationary"
finger = "index"
taxel = 1:8 

fs = 250; 

% Pre-allocatation
[n_samples, ~] = size(csvread(fullfile(folder, test, sprintf("%s_t%d.txt", finger, 0))));
all_signals = zeros(n_samples, length(taxel)); 

for t = taxel
    path = fullfile(folder, test, sprintf("%s_t%d.txt", finger, t-1)); 
    all_signals(:, t) = csvread(path);
end

all_signals = all_signals';  % #RIGA = (#TASSELLO-1) 

% integrale lungo le righe
int_tax = cumsum(all_signals, 2)/fs; 

% media dei primi 4 tasselli + integrale 
mean_signals = mean(all_signals(1:4, :)); 
int_mean_signal = cumsum(mean_signals)/fs; 

fig1 = figure(1);
fig1.WindowState = 'maximized';
clf

step = 40; 
save_max = 0;
subplot(211)
for t = taxel
    plot(all_signals(t, :)+t*step)
    grid on, hold on
    save_max = max(save_max, max(all_signals(t, :)+t*step));
    % plot(int_tax(t, :)+t*step, 'LineWidth', 2, 'Color','r')
end
xlim([0,n_samples])
title(sprintf("All signals"), 'FontWeight','normal')

subplot(212)
plot(mean_signals)
grid on, hold on
plot(int_mean_signal, 'LineWidth', 2, 'Color','r')
title(sprintf("Mean taxels [0-3] + Integral"), 'FontWeight','normal')
xlim([0,n_samples])

subplot(211)
text(0, 1.8*mean(all_signals(t, :)+t*step), sprintf('TEST = %s, FINGER = %s', test, finger),'Color','k', 'FontWeight','bold', FontSize=10)

folder_save = 'figures\test-all-mean4';   funcCreateFolder(folder_save)
fpath_save = fullfile(folder_save, sprintf('%s-%s.png', test, finger));
saveas(1, fpath_save);
close all


% %%% PLOT TASSELLI+INTEGRALI
% fig2 = figure(2);
% fig2.WindowState = 'maximized';
% clf
% 
% for t = taxel
% 
%     subplot(4, 2, t)
%     plot(all_signals(t, :))
%     hold on, grid on
%     plot(int_tax(t, :), 'LineWidth', 2, 'Color','r')
%     xlim([0,n_samples])
%     title(sprintf("Taxel %d", t), 'FontWeight','normal')
% 
% end
% 
% subplot(421)
% text(0, 2.2*max([max(all_signals(1, :)), max(int_tax(1, :))]), sprintf('TEST = %s, FINGER = %s', test, finger),'Color','k', 'FontWeight','bold', FontSize=10)
% 
% folder_save = 'figures\test-taxels';   funcCreateFolder(folder_save)
% fpath_save = fullfile(folder_save, sprintf('%s-%s.png', test, finger));
% saveas(2, fpath_save);
% close all
% 
% 
% %%% PLOT TEST THRESHOLD
% fig3 = figure(3); 
% fig3.WindowState = 'maximized';
% clf
% 
% threshold = 30;
% for t = taxel
%     subplot(4, 2, t)
%     plot(all_signals(t, :))
%     hold on, grid on
%     % plot(int_tax(t, :), 'LineWidth', 2, 'Color','r')
%     xlim([0, n_samples])
%     ylim([-threshold, threshold])
%     title(sprintf("Taxel %d", t), 'FontWeight','normal')
% end
% 
% subplot(421)
% text(0, 1.6*threshold, sprintf('TEST = %s, FINGER = %s', test, finger),'Color','k', 'FontWeight','bold', FontSize=10)
% 
% folder_save = 'figures\test-taxels-threshold';   funcCreateFolder(folder_save)
% fpath_save = fullfile(folder_save, sprintf('%s-%s-th%d.png', test, finger, threshold));
% saveas(3, fpath_save);
% close all
