%% GRASP DETECTION considerando il singolo tassello
clc, clear all

folder = "signals_txt";
test = "cube-sliding";

filter_bool = 0;
f = funcUseFilter(filter_bool);

save_fig = 1;

%%% Determine length of the acquisition
% [n_samples, ~] = size(csvread(fullfile(folder, test, sprintf("%s-t%d.txt", sprintf("%s%s", f, "thumb") , 0))));
taxels = 1:8; n_taxels = length(taxels);
n_tare = 1000;

%%% BP filter
fs = 250;
fc = [0.05, 3];
order = 2;
type = 'bandpass';

finger_name = ["THUMB "; "INDEX "; "MIDDLE"]; 
n_fingers = size(finger_name, 1);

%%% Extract txt
finger = sprintf("%s%s", f, "thumb");
raw_thumb = funcLoadTxt(folder, test, finger, taxels);
finger = sprintf("%s%s", f, "index");
raw_index = funcLoadTxt(folder, test, finger, taxels);
finger = sprintf("%s%s", f, "middle");
raw_middle = funcLoadTxt(folder, test, finger, taxels);

raw_all = [raw_thumb; raw_index; raw_index];
filt_all = zeros(size(raw_all));

n_samples = size(raw_all,2);

for line = 1:3*n_taxels
    if filter_bool % filtro già applicato onlone
        filt_all(line, :) = raw_all(line, :);
    else % filtro non applicato online
        filt_all(line, :) = funcButter(raw_all(line, :), order, fc, fs, type);
    end
end
abs_all = abs(filt_all);
mean_abs = mean(abs_all, 2);
std_abs = std(abs_all, 0, 2);

mean_tare = mean(abs_all(:, 1:n_tare), 2);
std_tare = std(abs_all(:, 1:n_tare), 0, 2);

%% OPEN FIGURES
%%% Open figured
for n_f = 1:n_fingers
        fig = figure(n_f);
        fig.WindowState = 'maximized';
        clf
end
disp("Figures opened")
disp('   ')

%% THRESHOLD FISSO
disp("THRESHOLD FISSO")
th_start_grasp = ones(size(raw_all, 1), 1)*1.8;
th_stop_grasp = th_start_grasp/1.5;

th_samples = 1500;

[grasp_region, start_grasp, stop_grasp] = funcThresholdAlgorithm(filt_all, abs_all, @funcShift, ...
    taxels, n_fingers, n_tare, ...
    th_start_grasp, th_stop_grasp, th_samples);

% %%%%%%%%%%%%%%%%%%%%%%%% ALGORITMO (TASSELLO x TASSELLO)
% 
% %%% PRE ALLOCAZIONI 
% grasp_region = NaN(size(abs_all));  %% [24xN] 1 se sono in grasp, 0 se non sono in grasp
% grasp_region(:,1) = 0;
% 
% counter_release = NaN(size(abs_all,1), 1); %% [24x1] conto il grasp per ogni tassello 
% counter_grasp = zeros(size(abs_all,1), 1); %% [24x1] conto i grasp identificati per ogni tassello 
% 
% start_grasp = NaN(size(abs_all)); %% [24xN] 1 al campione di inizio grasp, altrimenti Nan
% stop_grasp = NaN(size(abs_all)); %% [24xN] 1 al campione di fine grasp, altrimenti Nan
% % taxel_grasp = NaN(size(abs_all)); %% [24xn] 
% 
% %%% Matrice FIFO (ad ogni ciclo, shift a sinistra e inserisco il valore corrente)
% tare_filt= NaN(size(filt_all, 1), n_tare); 
% tare_filt(:, end) = filt_all(:, 1);
% tare_abs= zeros(size(abs_all, 1), n_tare); 
% tare_abs(:, end) = abs_all(:, 1); 
% 
% for step = 2:n_samples
%     current_abs = abs_all(:, step);  %% estraggo valori al campione corrente
% 
%     tare_filt = funcShift(tare_filt, filt_all(:, step));  %% shift filtrato
%     tare_abs  = funcShift(tare_abs,  abs_all(:, step));   %% shoft valore assoluto
% 
%     for t = taxels
%         for n_finger = 1:3
%             line = t+8*(n_finger-1); %% estraggo linea corrispondente al tassello della mano corrente 
% 
%             %%%%%% NO GRASP al campione precedente
%             if grasp_region(line, step-1) == 0  
% 
%                 %%%% START GRASP
%                 if current_abs(line)>th_start_grasp 
%                     grasp_region(line, step) = 1;                       %% salvo "1" nel campione corrente alla linea corrispondente 
%                     % taxel_grasp(line, n) = t;                         %% salvo il tassello di inizio grasp alla linea corrispondente
%                     start_grasp(line, step) = step;                     %% salvo il campione di fine grasp alla linea corrispondente
%                     counter_grasp(line) = counter_grasp(line)+1;        %% counto grasp alla linea corrispondente
% 
%                 %%%% NO GRASP
%                 else 
%                     grasp_region(line, step) = 0;  %% salvo "0" nel campione corrente alla linea corrispondente
%                 end
% 
%             %%%%%% GRASP al campione precedente
%             else 
%                 %%%% POTENZIALE FINE GRASP
%                 if current_abs(line)<th_stop_grasp 
% 
%                     counter_release(line) = counter_release(line)+1; %% inizio a contare i campioni sotto soglia
% 
%                     %%% FINE GRASP
%                      if counter_release(line)>th_samples    %% counter al valoredi soglia
%                         grasp_region(line,step-th_samples:step) = 0;            %% salvo "0" nei 1500 campione prima del corente alla linea corrispondente 
%                         counter_release(line) = 0;                              %% reset counter alla linea corrispondente 
%                         stop_grasp(line, step-th_samples) = step-th_samples;    %% salvo il campione-1500 di fine grasp alla linea corrispondente
% 
%                     %%% ANCORA IN GRASP
%                     else
%                         grasp_region(line, step) = 1;  %% salvo "1" nel campione corrente alla linea corrispondente 
%                     end
% 
%                 %%%% ANCORA IN GRASP
%                 else 
%                     counter_release(line) = 0;      %% reset counter perchè ancora in grasp 
%                     grasp_region(line, step) = 1;      %% salvo "1" nel campione corrente alla linea corrispondente 
%                 end
%             end
%         end
%     end
% end
% 
% disp("Algorithm ended")

color = 'r';
for t = taxels
    for n_f = 1:n_fingers

        fig = figure(n_f);
        line = t+8*(n_f-1);

        % disp([t, n_finger, line])
        subplot(4, 2, t), hold on, grid on
        % title(sprintf('Finger %d - Taxel %d [%d]', n_finger, t, line))

        % plot(raw_all(line, :))
        % plot(filt_all(line, :), 'LineWidth', 1)
        plot(abs_all(line, :))
        xlim([0, n_samples])
        ylim([-10*mean_abs(line), 15*mean_abs(line,1)])

        yline(th_start_grasp(line), '-',  Color=color, LineWidth=1.5)
        yline(th_stop_grasp(line),  '--', Color=color, LineWidth=1.5)
        plot(grasp_region(line, :)*3-10*mean_abs(line,1), Color=color, LineWidth=2)
        
        text(n_samples, grasp_region(line, end)*3-10*mean_abs(line,1), sprintf(" Th. FISSO = %.2f", th_start_grasp(line)), Color=color, ...
            HorizontalAlignment='left', FontSize=8)
        

        counter_grasp = 0;
        for step = 1:n_samples    
             if ~isnan(start_grasp(line, step))
                % N. grasp
                counter_grasp = counter_grasp+1;
                text(step, grasp_region(line, step)*3-10*mean_abs(line,1), sprintf("  %d", counter_grasp), Color=color, ...
                    HorizontalAlignment='left', VerticalAlignment='top', FontSize=8, FontWeight='bold')
             end

            if ~isnan(stop_grasp(line, step))
                % Regione della tara
                xregion(stop_grasp(line, step)+th_samples-n_tare, stop_grasp(line, step)+th_samples, 'FaceColor', color , 'FaceAlpha', 0.2)
          
            end
        end
    end
end

disp("Figures are plotted")
disp(" ")

%% THRESHOLD CON STD

disp("THRESHOLD CON STD")
th_start_grasp = mean_tare+5*std_tare;
th_stop_grasp = th_start_grasp;

th_samples = 1500;

[grasp_region, start_grasp, stop_grasp] = funcThresholdAlgorithm(filt_all, abs_all, @funcShift, ...
    taxels, n_fingers, n_tare, ...
    th_start_grasp, th_stop_grasp, th_samples);

% %%%%%%%%%%%%%%%%%%%%%%%% ALGORITMO (TASSELLO x TASSELLO)
% 
% %%% PRE ALLOCAZIONI 
% grasp_region = NaN(size(abs_all));  %% [24xN] 1 se sono in grasp, 0 se non sono in grasp
% grasp_region(:,1) = 0;
% 
% counter_release = NaN(size(abs_all,1), 1); %% [24x1] conto il grasp per ogni tassello 
% counter_grasp = zeros(size(abs_all,1), 1); %% [24x1] conto i grasp identificati per ogni tassello 
% 
% start_grasp = NaN(size(abs_all)); %% [24xN] 1 al campione di inizio grasp, altrimenti Nan
% stop_grasp = NaN(size(abs_all)); %% [24xN] 1 al campione di fine grasp, altrimenti Nan
% % taxel_grasp = NaN(size(abs_all)); %% [24xn] 
% 
% %%% Matrice FIFO (ad ogni ciclo, shift a sinistra e inserisco il valore corrente)
% tare_filt= NaN(size(filt_all, 1), n_tare); 
% tare_filt(:, end) = filt_all(:, 1);
% tare_abs= zeros(size(abs_all, 1), n_tare); 
% tare_abs(:, end) = abs_all(:, 1); 
% 
% for step = 2:n_samples
%     current_abs = abs_all(:, step);  %% estraggo valori al campione corrente
% 
%     tare_filt = funcShift(tare_filt, filt_all(:, step));  %% shift filtrato
%     tare_abs  = funcShift(tare_abs,  abs_all(:, step));   %% shoft valore assoluto
% 
%     for t = taxels
%         for n_finger = 1:3
%             line = t+8*(n_finger-1); %% estraggo linea corrispondente al tassello della mano corrente 
% 
%             %%%%%% NO GRASP al campione precedente
%             if grasp_region(line, step-1) == 0  
% 
%                 %%%% START GRASP
%                 if current_abs(line)>th_start_grasp 
%                     grasp_region(line, step) = 1;                       %% salvo "1" nel campione corrente alla linea corrispondente 
%                     % taxel_grasp(line, n) = t;                         %% salvo il tassello di inizio grasp alla linea corrispondente
%                     start_grasp(line, step) = step;                     %% salvo il campione di fine grasp alla linea corrispondente
%                     counter_grasp(line) = counter_grasp(line)+1;        %% counto grasp alla linea corrispondente
% 
%                 %%%% NO GRASP
%                 else 
%                     grasp_region(line, step) = 0;  %% salvo "0" nel campione corrente alla linea corrispondente
%                 end
% 
%             %%%%%% GRASP al campione precedente
%             else 
%                 %%%% POTENZIALE FINE GRASP
%                 if current_abs(line)<th_stop_grasp 
% 
%                     counter_release(line) = counter_release(line)+1; %% inizio a contare i campioni sotto soglia
% 
%                     %%% FINE GRASP
%                     if counter_release(line)>th_samples    %% counter al valoredi soglia
%                         grasp_region(line,step-th_samples:step) = 0;            %% salvo "0" nei 1500 campione prima del corente alla linea corrispondente 
%                         counter_release(line) = 0;                              %% reset counter alla linea corrispondente 
%                         stop_grasp(line, step-th_samples) = step-th_samples;    %% salvo il campione-1500 di fine grasp alla linea corrispondente
% 
%                         %%% ANCORA IN GRASP
%                     else
%                         grasp_region(line, step) = 1;  %% salvo "1" nel campione corrente alla linea corrispondente 
%                     end
% 
%                 %%%% ANCORA IN GRASP
%                 else 
%                     counter_release(line) = 0;      %% reset counter perchè ancora in grasp 
%                     grasp_region(line, step) = 1;      %% salvo "1" nel campione corrente alla linea corrispondente 
%                 end
%             end
%         end
%     end
% end
% 
% disp("Algorithm ended")

color = 'b'; 
for t = taxels
    for n_f = 1:n_fingers

        fig = figure(n_f);
        line = t+8*(n_f-1);

        subplot(4, 2, t), hold on, grid on
        % title(sprintf('Taxel %d', t))

        % plot(raw_all(line, :))
        % plot(filt_all(line, :), 'LineWidth', 1)
        % plot(abs_all(line, :))

        xlim([0, n_samples])
        ylim([-10*mean_abs(line), 15*mean_abs(line,1)])

        yline(th_start_grasp(line), '-',  Color=color, LineWidth=1.5)
        yline(th_stop_grasp(line),  '--', Color=color, LineWidth=1.5)

        plot(grasp_region(line, :)*2-5*mean_abs(line,1), Color='b', LineWidth=2)
        text(n_samples, grasp_region(line, end)*3-5*mean_abs(line,1), sprintf(" Th. STD = %.2f", th_start_grasp(line)), Color=color, ...
            HorizontalAlignment='left', FontSize=8)

        counter_grasp = 0;
        for step = 1:n_samples    

             if ~isnan(start_grasp(line, step))
                % N. grasp
                counter_grasp = counter_grasp+1;
                text(step, grasp_region(line, step)*3-5.7*mean_abs(line,1), sprintf("   %d", counter_grasp), Color=color, ...
                    HorizontalAlignment='left', VerticalAlignment='top', FontSize=8, FontWeight='bold')
             end

            if ~isnan(stop_grasp(line, step))
                % Regione della tara
                xregion(stop_grasp(line, step)+th_samples-n_tare, stop_grasp(line, step)+th_samples, 'FaceColor', color , 'FaceAlpha', 0.2)
            end
        end
    end
end


%% TITLE 

for n_f = 1:n_fingers
    fig = figure(n_f);
    
    line = 1+8*(n_f-1);
    subplot(4,2,1)
    text(n_samples+2000, 1*max(abs_all(line, :)), sprintf('%s - %s', test, finger_name(n_f, :)),'Color','k', 'FontWeight','bold', FontSize=10)
    text(n_samples+2000, 0.8*max(abs_all(line, :)), sprintf('Th Samples = %d', th_samples),'Color','k', 'FontWeight','normal', FontSize=8)

    for t = taxels
        subplot(4, 2, t), hold on, grid on
        title(sprintf('Taxel %d', t))
    end

end

disp("Figures are plotted")
disp(" ")


%% SAVE FIGURE
for n_f = 1:n_fingers
    fig = figure(n_f);
    
    folder_save = 'figures/fig-grasp-det-taxels-singles';
    name_figure = sprintf('%s-%s.png', test, finger_name(n_f,:));

    funcSaveFigure(save_fig, n_f, folder_save, name_figure, @funcCreateFolder)
    close 
end

disp("Figures are saved")