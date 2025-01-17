%% Comparison filters
clc, clear all

order = 2; 
fc_inf = 10; 
fc_sup = 40;
fs = 250; 

%% BUTTERWORTH 
% Progettazione del filtro
[b, a] = butter(order,[fc_inf, fc_sup]/(fs/2),"bandpass");

% Calcolo della risposta in frequenza
[freqResp, freq] = freqz(b, a, 8192, fs);

% Grafico della funzione di trasferimento (modulo)
figure(1), clf
subplot(2,1,1)
plot(freq, abs(freqResp), 'LineWidth', 2); % Modulo della funzione di trasferimento
title('BUTTERWORTH');
subtitle('Risposta in Ampiezza')
xlabel('Frequenza (Hz)');
ylabel('Ampiezza');
grid on;


%% ELLIPTIC
rp = 1;   % Ripple massimo in banda passante (in dB)
rs = 40;  % Attenuazione minima nella banda attenuata (in dB)

[b, a] = ellip(order, rp, rs, [fc_inf, fc_sup] / (fs / 2), 'bandpass');

% Calcolo della risposta in frequenza
[freqResp, freq] = freqz(b, a, 8192, fs);

% Grafico della funzione di trasferimento (modulo)
subplot(2,1,2)
plot(freq, abs(freqResp), 'LineWidth', 2); % Modulo della funzione di trasferimento
title('ELLIPTIC');
subtitle('Risposta in Ampiezza')
xlabel('Frequenza (Hz)');
ylabel('Ampiezza');
grid on;

