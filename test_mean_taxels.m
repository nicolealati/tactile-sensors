% TEST  MEDIA TASSELLI
clc, clear all

folder = "signals_txt";
test = "cube-stationary"
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


%%% 
all_signals = all_signals';  % #RIGA = (#TASSELLO-1) 

% integrale lungo le righe
int_tax = cumsum(all_signals, 2)/fs; 

% media dei primi 4 tasselli + integrale 
mean_signals = mean(all_signals(1:4, :)); 
int_mean_signal = cumsum(mean_signals)/fs; 

% fig1 = figure(1); 
% fig1.WindowState = 'maximized';
% clf
% 
% subplot(211)
% range = 2000:8000; 
% for t = taxel
%     plot(all_signals(t, :)+t*30)
%     grid on, hold on 
%     plot(int_tax(t, :)+t*30, 'LineWidth', 2, 'Color','r')
% end  
% title(sprintf("%s - %s ", test, finger))
% 
% subplot(212)
% plot(mean_signals)
% grid on, hold on
% plot(int_mean_signal, 'LineWidth', 2, 'Color','r')

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
% 
%     title(sprintf("Taxel %d", t))
% 
% end
% 
% folder_save = 'figures\test-taxels';   funcCreateFolder(folder_save)
% fpath_save = fullfile(folder_save, sprintf('%s-%s.png', test, finger));
% saveas(2, fpath_save);


fig3 = figure(3); 
fig3.WindowState = 'maximized';
clf

threshold = 30
for t = taxel
    subplot(4, 2, t)
    plot(all_signals(t, :))
    hold on, grid on
    plot(int_tax(t, :), 'LineWidth', 2, 'Color','r')
    xlim([0,n_samples])
    ylim([-threshold, threshold])
    title(sprintf("Taxel %d", t))
end

% folder_save = 'figures\test-taxels-threshold';   funcCreateFolder(folder_save)
% fpath_save = fullfile(folder_save, sprintf('%s-%s-th%d.png', test, finger, threshold));
% saveas(3, fpath_save);




% signal_raw = csvread(path);
% signal_integral = cumsum(signal_raw);
% 
% order_raw = 1;
% framelen_raw = 201;
% 
% fs = 250;
% estimated_delay = framelen_raw/fs/2
% 
% signal_filt = sgolayfilt(signal_raw, order_raw, framelen_raw);
% signal_filt_int = cumsum(signal_filt);
% 
% [b,a] = butter(2, [0.02 0.5]/(fs/2), 'bandpass');
% signal_bp = filter(b,a,signal_raw);
% signal_bp_demean = signal_bp - mean(signal_bp);
% integral_bp_demean = cumsum(signal_bp_demean);
% 
% fig3 = figure(3);
% fig3.WindowState = 'maximized';
% 
% clf
% subplot(2,2,1)
% plot(signal_raw, 'LineWidth', 0.5)
% hold on, grid on
% plot(signal_filt, 'LineWidth', 2, 'Color', 'r')
% plot(signal_integral/fs, 'LineWidth', 1.5)
% plot(signal_filt_int/fs, 'LineWidth', 2)
% legend('raw', 'raw-filt', 'integral', 'filt-integral')
% % xlim([0, 50000])
% % 
% % title(sprintf("%s - %s - (taxel %d)", test, finger, taxel))
% % 
% % subplot(2,2,3), hold on, grid on
% % plot(signal_filt, 'LineWidth', 2, 'Color', 'r')
% % plot(integral_bp_demean/fs,'LineWidth', 2, 'LineStyle',':', 'Color','k')
% % plot(signal_filt_int/fs, 'LineWidth', 2)
% % % 
% % % plot(signal_integral/fs, 'LineWidth', 1.5)
% % % plot(signal_filt_int/fs, 'LineWidth', 2)
% % 
% % xlim([0, 50000])
% % legend('filtrato', 'integrale-filtr', 'integral-BP')
% % xlim([0, 50000])
% % 
% % subplot(2,2,2)
% % plot(signal_raw, 'LineWidth', 0.5)
% % hold on, grid on
% % plot(signal_integral/fs, 'LineWidth', 1.5)
% % xlim([0, 50000])
% % legend('raw', 'integral-raw')
% % subtitle("Versione senza filtro")
% % 
% % subplot(2,2,4)
% % plot(signal_filt, 'LineWidth', 2, 'Color', 'r')
% % hold on, grid on
% % plot(signal_filt_int/fs, 'LineWidth', 2)
% % % plot(integral_bp_demean/fs,'LineWidth', 2, 'LineStyle',':', 'Color','k')
% % xlim([0, 50000])
% % legend('filtrato', 'integrale-del-filtrato')
% % 
% % subtitle("Versione con filtro Savitzky-Golay")
% 
