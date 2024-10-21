% TEST IDENTIFICAZIONE GRASP
clc, clear all

folder = "signals_txt";
test = "sphere-sliding"
finger = "thumb"
taxel = 1:8

grasp = 0;
counter = 0;
threshold_ampiezza_vect = [15, 10];
threshold_samples = 1500;
n_grasp = 1;

fs = 250;

for threshold_ampiezza = threshold_ampiezza_vect
    % Pre-allocatation
    [n_samples, ~] = size(csvread(fullfile(folder, test, sprintf("%s_t%d.txt", finger, 0))));
    all_signals = zeros(n_samples, length(taxel));

    for t = taxel
        path = fullfile(folder, test, sprintf("%s_t%d.txt", finger, t-1));
        all_signals(:, t) = csvread(path);
    end

    %%%
    all_signals = all_signals';  % #RIGA = (#TASSELLO-1)
    filt_signals = zeros(size(all_signals));
    fc = [0.1 5];
    order = 2;

    for t = taxel
        [b,a] = butter(order,fc/(fs/2),'bandpass');
        filt_signals(t, :) = filter(b,a,all_signals(t, :));
    end


    abs_signal = abs(filt_signals);


    fig3 = figure(3);
    fig3.WindowState = 'maximized';
    clf

    for t = taxel
        subplot(4, 2, t)
        plot(filt_signals(t, :))
        hold on, grid on
        plot(abs_signal(t, :))
        xlim([0,n_samples])
        % title(sprintf("Taxel %d", t))
        yline(threshold_ampiezza,'r--', 'LineWidth', 2)
    end

    for n = 1:n_samples
        current_value = abs_signal(:, n);

        if grasp == 0
            for t = taxel % per ogni tassello
                if current_value(t) > threshold_ampiezza
                    grasp = 1;
                    taxel_grasp = t;
                    start_grasp = n;

                    subplot(4,2,taxel_grasp)
                    xline(start_grasp, 'b', 'LineWidth', 1)
                end
            end
        else
            if current_value(taxel_grasp) < threshold_ampiezza/2
                counter = counter + 1;
            else
                counter = 0;
            end
            if counter > threshold_samples
                grasp = 0;
                counter = 0;
                for t = taxel
                    subplot(4,2,t)
                    xline(n, 'k', 'LineWidth', 1)
                    ylim([-50, 50])
                    text(n+200, -45, sprintf('%d', n_grasp),'Color','k','FontWeight','bold', FontSize=8)
                    xregion(n-1000, n, 'FaceColor','g', 'FaceAlpha', 0.2)
                    xregion(start_grasp, n, 'FaceColor','b', 'FaceAlpha', 0.1)
                    text(round(n+start_grasp)/2, 45, 'GRASP','Color','b','FontWeight','normal', ...
                        HorizontalAlignment='center', FontSize=6)
                    text(round(n_samples/2), 60, sprintf('Taxel %d', t), HorizontalAlignment='center', FontSize=9)
                end

                subplot(4,2,taxel_grasp)
                xline(n, 'r', 'LineWidth', 2)
                text(n+200, -45 , sprintf('%d', n_grasp),'Color','r', 'FontWeight','bold', FontSize=8)
                n_grasp = n_grasp+1 ;

            end
        end
    end

    subplot(421)
    text(0, 90, sprintf('TEST = %s, FINGER = %s', test, finger),'Color','k', 'FontWeight','bold', FontSize=10)
    text(0, 75, sprintf('Th Samples = %d, Th Amp = %d', threshold_samples, threshold_ampiezza), FontSize=8)


    folder_save = sprintf('figures/grasp-identification/s%d-a%d', threshold_samples, threshold_ampiezza);
    funcCreateFolder(folder_save)
    fpath_save = fullfile(folder_save, sprintf('%s-%s-s%d-a%d.png', test, finger, threshold_samples, threshold_ampiezza));
    saveas(3, fpath_save);
    close all
end