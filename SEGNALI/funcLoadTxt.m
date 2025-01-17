function  signals = funcLoadTxt(folder, test, finger, taxel)

% disp(fullfile(folder, test, sprintf("%s-t%d.txt", finger , 0)))

[n_samples, ~] = size(csvread(fullfile(folder, test, sprintf("%s-t%d.txt", finger , 0))));
signals = zeros(length(taxel), n_samples);

for t = taxel

    % Percorso cartella
    txt_path = fullfile(folder, test, sprintf("%s-t%d.txt", finger, t-1));

    % Load del segnale (n_samples x 1
    temp_signal = csvread(txt_path);
    signals(t, :) = round(temp_signal');

end

end

