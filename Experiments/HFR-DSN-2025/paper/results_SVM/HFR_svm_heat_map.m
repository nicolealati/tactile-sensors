clc, clear all, close all

bool_save = 1
filename = 'SVM_accuracy.xlsx';
groups = {'raw signal', 'STFT', 'DWT marg.'};

font = 'Helvetica';
size = 10;

dimension = [100, 100, 300, 100]; 
map_type = 'summer'; 

%%% PRESSURE (contact vs non-contact)
excel_sheet = "PRESSURE"
without_ar10 = readmatrix(filename, Sheet=excel_sheet , Range="B3:D3")
with_ar10 = readmatrix(filename,  Sheet=excel_sheet, Range="B4:D4")

data = [without_ar10; with_ar10]; 

name_figure = 'HFR_svm_contact_heat_map.pdf';
figure('Position', dimension); % [x, y, larghezza, altezza]

features = {'raw s.', 'STFT', 'DWT marg.'};
groups = {'Human f.', 'Robotic f.'};

h = heatmap(groups, features, data');
h.FontName = font;
h.FontSize = size;
h.CellLabelFormat = '%.2f';
h.CellLabelColor = 'black';

colormap(h, map_type); 
caxis([0 100]);

% Save 
if bool_save
    disp('Saving figure...')
    exportgraphics(gcf, name_figure, 'ContentType', 'vector');
    disp('SAVED')
end

%%% PRESSURE (livelli)

excel_sheet = "PRESSURE"
without_ar10 = readmatrix(filename, Sheet=excel_sheet , Range="B8:D8")
with_ar10 = readmatrix(filename,  Sheet=excel_sheet, Range="B9:D9")
data = [without_ar10; with_ar10]; 

name_figure = 'HFR_svm_levels_heat_map.pdf';
figure('Position', dimension); % [x, y, larghezza, altezza]

features = {'raw s.', 'STFT', 'DWT marg.'};
groups = {'Human f.', 'Robotic f.'};

h = heatmap(groups, features, data');
h.FontName = font;
h.FontSize = size;
h.CellLabelFormat = '%.2f';
h.CellLabelColor = 'black';

colormap(h, map_type); 
caxis([0 100]);

% Save 
if bool_save
    disp('Saving figure...')
    exportgraphics(gcf, name_figure, 'ContentType', 'vector');
    disp('SAVED')
end

%%% SLIDING

excel_sheet = "SLIDING"
rest_vs_sliding = readmatrix(filename, Sheet=excel_sheet , Range="B3:D3")
rest_vs_linear = readmatrix(filename,  Sheet=excel_sheet, Range="B4:D4")
rest_vs_rotational = readmatrix(filename, Sheet=excel_sheet , Range="B5:D5")
rest_vs_lin_vs_rot = readmatrix(filename,  Sheet=excel_sheet, Range="B6:D6")

% data = [rest_vs_sliding; rest_vs_linear; rest_vs_rotational; rest_vs_lin_vs_rot]'; 
% 
% dimension(end) = dimension(end)+50; 
% name_figure = 'HFR_svm_sliding_heat_map.pdf';
% figure('Position', dimension); % [x, y, larghezza, altezza]

% h = heatmap(groups, features, data);
% h.FontName = font;
% h.FontSize = size;
% h.CellLabelFormat = '%.2f';
% h.CellLabelColor = 'black';
% 
% colormap(h, map_type); 
% caxis([0 100]);

data = [rest_vs_sliding; rest_vs_linear; rest_vs_rotational; rest_vs_lin_vs_rot]'; 

dimension(3) = dimension(3)+380
name_figure = 'HFR_svm_sliding_heat_map.pdf';
figure('Position', dimension);  % [x, y, larghezza, altezza]

features = {'raw s.', 'STFT', 'DWT marg.'};
groups = {'rest vs. slid', 'rest vs. lin', 'rest vs. rot', 'rest vs. lin vs. rot'};

h = heatmap(groups, features, data);  % dati NON trasposti perché vogliamo features su Y, gruppi su X
h.FontName = font;
h.FontSize = size;
h.CellLabelFormat = '%.2f';
h.CellLabelColor = 'black';

colormap(h, map_type);
caxis([0 100]);

% Save 
if bool_save
    disp('Saving figure...')
    exportgraphics(gcf, name_figure, 'ContentType', 'vector');
    disp('SAVED')
end


