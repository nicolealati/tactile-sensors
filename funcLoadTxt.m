function  signals = funcLoadTxt(folder, test, finger, taxel, n_samples)

signals = zeros(length(taxel), n_samples);

for t = taxel

    % Definizione del path in cartella
    txt_path = fullfile(folder, test, sprintf("%s_t%d.txt", finger, t-1));

    % Salvo il segnale in variabile temporanea
    temp_signal = csvread(txt_path);

    % Salvo segnali (grezzo, filtrato, valore assoluto del filtrato
    signals(t, :) = round(temp_signal');
end

end

