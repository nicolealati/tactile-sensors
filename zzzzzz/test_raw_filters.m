%% ALGORITMO PER L'IDENTIFICAZIONE DEL GRASP DA SEGNALE
%%%% THRESHOLD TASSELLO PER TASSELLO
clc, clear all

folder = "signals_txt";
test = "cylinder-filter-2";
finger = "middle";

filter_bool = 0;
f = funcUseFilter(filter_bool);
finger = sprintf("%s%s", f, finger);

disp(' '), disp([test, finger])
taxels = 1:8; % In python, da 0 a 7
fs = 250;

%% RAW 
[n_samples, ~] = size(csvread(fullfile(folder, test, sprintf("%s-t%d.txt", finger, 0))));

% Pre-allocazione della matrice (8x n_samples) per i segnali
all_signals = zeros(length(taxels), n_samples);

for t = taxels
    txt_path = fullfile(folder, test, sprintf("%s-t%d.txt", finger, t-1));
    temp_signal = csvread(txt_path);
    all_signals(t, :) = temp_signal';
end

n_figure = 1;
fig1 = figure(n_figure);
fig1.WindowState = 'maximized';
clf

% Per ogni tassello, plot del segnale filtrato e del valore assoluto
for t = taxels
    subplot(4, 2, t), hold on, grid on
    xlim([0,n_samples])
    plot(all_signals(t, :))
    title(sprintf('Taxel %d', t))
    % text(round(n_samples/2), limit*1.2, sprintf('Taxel %d', t), 'FontWeight', 'normal', FontSize=9, ...
    %     HorizontalAlignment='center')
end

% Filtro passa-banda
fc = [0.05 5];
order = 2;
type = 'bandpass';

limit = 30;
if filter_bool
    limit = limit/5;
end
  
filt_BP = zeros(size(all_signals));

% #RIGA = #TASSELLO (TASSELLO-1 per Python)
for t = taxels
    % Applico filtro (se serve)
    if filter_bool % filtro già applicato onlone
        filt_BP(t, :) = all_signals(t, :);
    else % filtro non applicato online
        filt_BP(t, :) = funcButter(temp_signal', order, fc, fs, type);

        subplot(4,2,t)
        plot(filt_BP(t, :))
        plot(abs(filt_BP(t,:)))
        ylim([-10, 10])

        legend('raw', 'BP[0.05-5]', 'abs(BP)')

    end

end
disp('END')