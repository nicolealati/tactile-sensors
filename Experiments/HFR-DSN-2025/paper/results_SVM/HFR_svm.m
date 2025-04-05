clc, clear all, close all

bool_save = 1
name_figure = 'HFR_svm_results.pdf';

filename = 'SVM_accuracy.xlsx';
groups = {'raw signal', 'STFT', 'DWT'};

%figure('Position', [100, 100, 1000, 500]); % [x, y, larghezza, altezza]
%figure; set(gcf, 'Position', get(0, 'ScreenSize'));

% https://toolset.mrw.it/html/colori-del-web.html
pressure_colors = [0.8667 0.6784 0.8745;  % Viola chiaro
    0.6275 0.7686 1.0000]; % Azzurro (#A0C4FF)

% colors_sliding = [0.3922 0.5843 0.9294;  % Blu Cornflower (#6495ED)
%     1.0000 0.6471 0.0000;  % Arancione (#FFA500)
%     0.8039 0.3608 0.3608;  % Rosso Indiano (#CD5C5C)
%     0.4000 0.8039 0.6706]; % Verde Medio Acquamarina (#66CDAA)

colors_sliding = [0.5882 0.7176 1.0000;  % Blu Cornflower chiaro
    1.0000 0.8118 0.3725;  % Arancione chiaro
    0.9373 0.5608 0.5608;  % Rosso Indiano chiaro
    0.6000 0.8784 0.8039]; % Verde Medio Acquamarina chiaro

font = 'Helvetica';
size = 10;
size_legend = 8;

%% PRESSURE (contact vs non-contact)

excel_sheet = "PRESSURE"
without_ar10 = readmatrix(filename, Sheet=excel_sheet , Range="B3:D3")
with_ar10 = readmatrix(filename,  Sheet=excel_sheet, Range="B4:D4")

name_figure = 'HFR_svm_contact.pdf';
figure('Position', [100, 100, 400, 150]); % [x, y, larghezza, altezza]

b_pressure_1 = bar([without_ar10; with_ar10]', 'grouped');
for k = 1:length(b_pressure_1)
    b_pressure_1(k).FaceColor = pressure_colors(k, :);
end

set(gca, 'XTickLabel', groups, 'FontSize', size-2, 'FontName', font)
legend({'human finger', 'robotic finger'}, 'Location', 'northeastoutside', 'FontSize', size_legend, 'FontName', font)
%xlabel('Features', 'FontSize', size, 'FontName', font)
ylabel('Accuracy (%)', 'FontSize', size, 'FontName', font)
% title('Pressure Task', 'FontSize', size, 'FontWeight', 'bold', 'FontName', font)
% subtitle(["PRESSURE TASKS (contact)"; "               "] , 'FontSize', size, 'FontName', font)
ylim([0, 100])
% annotation('textbox', [0.055, 0.74, 0.1, 0.05], 'String', 'a', 'FontWeight', 'bold', ...
    % 'EdgeColor', 'none', 'FontSize', size+3, 'FontName', 'Times New Romans', 'HorizontalAlignment', 'left');

for i = 1:length(b_pressure_1)
    xtips = b_pressure_1(i).XEndPoints;
    ytips = b_pressure_1(i).YEndPoints;
    labels = string(round(b_pressure_1(i).YData, 1));% + "%";
    text(xtips, ytips + 2, labels, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
        'FontSize', size-2, 'FontName', 'Helvetica', 'FontWeight', 'normal');
end
box("off")

% annotation('textbox', [0.235, 0.96, 0.4, 0.05], 'String', 'Pressure Tasks', ...
% 'EdgeColor', 'none', 'FontSize', size, 'FontName', 'Helvetica', 'HorizontalAlignment', 'center', FontWeight='bold');

if bool_save
    disp('Saving figure...')
    exportgraphics(gcf, name_figure, 'ContentType', 'vector');
    disp('SAVED')
end

close
%% PRESSURE (livelli)

excel_sheet = "PRESSURE"
without_ar10 = readmatrix(filename, Sheet=excel_sheet , Range="B8:D8")
with_ar10 = readmatrix(filename,  Sheet=excel_sheet, Range="B9:D9")

name_figure = 'HFR_svm_level.pdf';
figure('Position', [100, 100, 400, 150]); % [x, y, larghezza, altezza]

bar_levels = bar([without_ar10; with_ar10]', 'grouped');
for k = 1:length(bar_levels)
    bar_levels(k).FaceColor = pressure_colors(k, :);
end
set(gca, 'XTickLabel', groups, 'FontSize', size-2, 'FontName', font)
legend({'human finger', 'robotic finger'}, 'Location', 'northeastoutside', 'FontSize', size_legend, 'FontName', font)
%xlabel('Features', 'FontSize', size, 'FontName', font)
ylabel('Accuracy (%)', 'FontSize', size, 'FontName', font)
% title('Pressure Task', 'FontSize', size, 'FontWeight', 'bold', 'FontName', font)
% subtitle(["PRESSURE TASK (force levels)"; "   "], 'FontSize', size, 'FontName', font)
ylim([0, 100])

% annotation('textbox', [0.495, 0.74, 0.1, 0.05], 'String', 'b', 'FontWeight', 'bold', ...
%     'EdgeColor', 'none', 'FontSize', size+3, 'FontName', 'Times New Romans', 'HorizontalAlignment', 'left');

for i = 1:length(bar_levels)
    xtips = bar_levels(i).XEndPoints;
    ytips = bar_levels(i).YEndPoints;
    labels = string(round(bar_levels(i).YData, 1));% + "%";
    text(xtips, ytips + 2, labels, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
        'FontSize', size-2, 'FontName', 'Helvetica', 'FontWeight', 'normal');
end
box("off")

if bool_save
    disp('Saving figure...')
    exportgraphics(gcf, name_figure, 'ContentType', 'vector');
    disp('SAVED')
end

close
%% SLIDING

excel_sheet = "SLIDING"
rest_vs_sliding = readmatrix(filename, Sheet=excel_sheet , Range="B3:D3")
rest_vs_linear = readmatrix(filename,  Sheet=excel_sheet, Range="B4:D4")
rest_vs_rotational = readmatrix(filename, Sheet=excel_sheet , Range="B5:D5")
rest_vs_lin_vs_rot = readmatrix(filename,  Sheet=excel_sheet, Range="B6:D6")

name_figure = 'HFR_svm_sliding.pdf';
figure('Position', [100, 100, 650, 150]); % [x, y, larghezza, altezza]

bar_sliding = bar([rest_vs_sliding; rest_vs_linear; rest_vs_rotational; rest_vs_lin_vs_rot]', 'grouped');
for k = 1:length(bar_sliding)
    bar_sliding(k).FaceColor = colors_sliding(k, :);
end
set(gca, 'XTickLabel', groups, 'FontSize', size-2, 'FontName', font)
%xlabel('Features', 'FontSize', size, 'FontName', font)
ylabel('Accuracy (%)', 'FontSize', size, 'FontName', font)
%title('Sliding Task', 'FontSize', size, 'FontWeight', 'bold', 'FontName', font)
% subtitle(["SLIDING Tfigure('Position', [100, 100, 200, 400]); % [x, y, larghezza, altezzaASK"; "  "], 'FontSize', size, 'FontName', font)
legend({'REST vs SLIDE', 'REST vs LIN', 'REST vs ROT', 'REST vs LIN vs ROT'}, ...
    'Location', 'northeastoutside', 'FontSize', size_legend, 'FontName', font)

% annotation('textbox', [0.055 0.27, 0.1, 0.05], 'String', 'c', 'FontWeight', 'bold', ...
%     'EdgeColor', 'none', 'FontSize', size+3, 'FontName', 'Times New Romans', 'HorizontalAlignment', 'left');

for i = 1:length(bar_sliding)
    xtips = bar_sliding(i).XEndPoints;
    ytips = bar_sliding(i).YEndPoints;
    labels = string(round(bar_sliding(i).YData, 1)) ; %+ "%";
    text(xtips, ytips + 2, labels, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
        'FontSize', size-2, 'FontName', 'Helvetica', 'FontWeight', 'normal');
end
box("off")

if bool_save
    disp('Saving figure...')
    exportgraphics(gcf, name_figure, 'ContentType', 'vector');
    disp('SAVED')
end
close