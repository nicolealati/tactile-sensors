% TEST Savitzky-Golay filtering
clc, clear all

folder = "signals_txt";
test = "Cylinder stationary"
finger = "index"
taxel = 6
path = fullfile(folder, test,sprintf("%s_t%d.txt", finger, taxel))

signal_raw = csvread(path);
signal_integral = cumsum(signal_raw);

order_raw = 1;
framelen_raw = 201;

fs = 250;
estimated_delay = framelen_raw/fs/2

signal_filt = sgolayfilt(signal_raw, order_raw, framelen_raw);
signal_filt_int = cumsum(signal_filt);

[b,a] = butter(2, [0.02 0.5]/(fs/2), 'bandpass');
signal_bp = filter(b,a,signal_raw);
signal_bp_demean = signal_bp - mean(signal_bp);
integral_bp_demean = cumsum(signal_bp_demean);

fig3 = figure(3);
fig3.WindowState = 'maximized';

clf
subplot(2,2,1)
plot(signal_raw, 'LineWidth', 0.5)
hold on, grid on
plot(signal_filt, 'LineWidth', 2, 'Color', 'r')
plot(signal_integral/fs, 'LineWidth', 1.5)
plot(signal_filt_int/fs, 'LineWidth', 2)
legend('raw', 'raw-filt', 'integral', 'filt-integral')
xlim([0, 50000])

title(sprintf("%s - %s - (taxel %d)", test, finger, taxel))

subplot(2,2,3), hold on, grid on
plot(signal_filt, 'LineWidth', 2, 'Color', 'r')
plot(integral_bp_demean/fs,'LineWidth', 2, 'LineStyle',':', 'Color','k')
plot(signal_filt_int/fs, 'LineWidth', 2)
% 
% plot(signal_integral/fs, 'LineWidth', 1.5)
% plot(signal_filt_int/fs, 'LineWidth', 2)

xlim([0, 50000])
legend('filtrato', 'integrale-filtr', 'integral-BP')
xlim([0, 50000])

subplot(2,2,2)
plot(signal_raw, 'LineWidth', 0.5)
hold on, grid on
plot(signal_integral/fs, 'LineWidth', 1.5)
xlim([0, 50000])
legend('raw', 'integral-raw')
subtitle("Versione senza filtro")

subplot(2,2,4)
plot(signal_filt, 'LineWidth', 2, 'Color', 'r')
hold on, grid on
plot(signal_filt_int/fs, 'LineWidth', 2)
% plot(integral_bp_demean/fs,'LineWidth', 2, 'LineStyle',':', 'Color','k')
xlim([0, 50000])
legend('filtrato', 'integrale-del-filtrato')

subtitle("Versione con filtro Savitzky-Golay")

% fig1 = figure(1);
% fig1.WindowState = 'maximized';
%
% clf
% plot(signal_raw, 'LineWidth', 0.5)
% hold on, grid on
% plot(signal_filt, 'LineWidth', 2, 'Color', 'r')
% plot(signal_integral/fs, 'LineWidth', 1.5)
% plot(signal_filt_int/fs, 'LineWidth', 2)
% plot(detrend(signal_filt_int)/fs, 'LineWidth', 2, 'LineStyle',':')
% legend('raw', 'raw-filt', 'integral', 'filt-integral','filt-integral-detrend')
% xlim([0, 50000])
%
% title(sprintf("%s - %s - (taxel %d)", test, finger, taxel))
%
%
% fig2 = figure(2);
% fig2.WindowState = 'maximized';
%
% clf
% subplot(2,1,1)
% plot(signal_raw, 'LineWidth', 0.5)
% hold on, grid on
% plot(signal_integral/fs, 'LineWidth', 1.5)
% xlim([0, 50000])
% legend('raw', 'integral-raw')
% subtitle("Senza filtro")
%
% subplot(2,1,2)
% plot(signal_filt, 'LineWidth', 2, 'Color', 'r')
% hold on, grid on
% plot(signal_filt_int/fs, 'LineWidth', 2)
% xlim([0, 50000])
% legend('filtrato', 'integrale-del-filtrato')
% subtitle("Con filtro Savitzky-Golay")
%
% title(sprintf("%s - %s - (taxel %d)", test, finger, taxel))