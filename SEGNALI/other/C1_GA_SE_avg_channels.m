clc, clear all, close all

bool_save_fig = 1;

%%% Folders' path
folder = 'data-zscores\3 data-grand_average_std_zscores';
folder_save = 'figures\A fig-ga_se_avg_channels-all_conditions';   funcCreateFolder(folder_save)

save_title = 'avg_ch_all_comb';

dimensions = [200, 100, 1500, 700];

%%%% Channels
load('z_ch_names.mat')
ROI_channels_indexes = [11:13, 18:20, 25:26, 31:33];
n_ROI_channels = length(ROI_channels_indexes);
% ROI_channels = ch_names(:, ROI_channels_indexes);
% grid_channels = [1:8, 10:12];

%%% Plot setting
n_figure = 1;
fig = figure(n_figure);
% fig.WindowState = "maximized";
fig.Position = dimensions; 

x_label = 'Time (s)';
y_label = 'Z-scores (a.u.)';
y_limit_sup = 0.7;
y_limit_inf = -0.35;
step = 0.1;
factor = 1;

load('z_color_line_plot.mat')
load('z_color_area_plot.mat')
line_width = 1.2;
face_alpha = 0.15;

% start_region_sep = 0.03;
% stop_region_sep  = 0.32;

subplot(2,4, [1 2 3, 5, 6, 7])
conditions = 1:6;
for cond = conditions

    struct = load(fullfile(folder, sprintf('comb%d-ga_std_zscores.mat', cond)));

    %%%% Extract important times
    onset_baseline = struct.onset_baseline;
    onset_stim     = struct.onset_stim;
    onset_rest     = onset_stim +2;

    %%%% Extract GA across subject 
    data_avg = struct.data_ga_zscores;   
    
    %%%% Limit time axis PRE-STIM
    time_plot_indexes = find(struct.time >= onset_baseline & struct.time <= onset_stim + 2);
    time_plot = struct.time(time_plot_indexes);
    
    %%%% Extract GA for PRE_STIM
    data_avg = data_avg(ROI_channels_indexes, time_plot_indexes);
    
    %%%% Calculate GA average across channels and SE of GA across channels
    all_avg = mean(data_avg, 1);
    all_se = std(data_avg)/sqrt(n_ROI_channels);

    hold on

    fill([time_plot, fliplr(time_plot)], ...
        [ all_avg * factor + all_se * factor, ...
        fliplr(all_avg * factor - all_se * factor)], ...
        color_area_plot(cond, :), 'FaceAlpha', face_alpha, 'EdgeColor','none');

    plot(time_plot, all_avg * factor, ...
        'LineWidth', line_width, 'Color', color_line_plot(cond, :)')

end %%% end conditions
xline(onset_stim, '--',  'LineWidth', 0.5)

title('   ')
subtitle('Grand average evolution', 'FontWeight', 'bold')

% xregion(start_region_sep, stop_region_sep, 'FaceColor','g', 'FaceAlpha', 0.2)
% 
% for x = [50, 150, 300]
%     xline(x*1e-3,'-', 'Color', 'b', 'LineWidth', 0.8)
%     text(x*1e-3, 0.75, sprintf('P%d', x), 'Color', 'b', 'FontSize', 4, 'HorizontalAlignment', 'center')
%     xregion((x-20)*1e-3, (x+20)*1e-3, 'FaceColor','b', 'FaceAlpha', 0.1)
% end
% 
% for x = 200
%     xline(x*1e-3, '-', 'Color', 'r', 'LineWidth', 0.8)
%     text(x*1e-3, -0.75, sprintf('N%d', x), 'Color', 'r', 'FontSize', 4, 'HorizontalAlignment', 'center')
%     xregion((x-20)*1e-3, (x+20)*1e-3, 'FaceColor','r', 'FaceAlpha', 0.1)
% end

text(0.65, 0.93,'STIM', 'Units', 'normalized', 'FontSize', 12, 'Color', '#636363', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
text(0.10, 0.93,'PRE ', 'Units', 'normalized', 'FontSize', 12, 'Color', '#636363', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');

ylabel(y_label)
xlabel(x_label)
ylim([y_limit_inf, y_limit_sup])

yticks([y_limit_inf,  0, y_limit_sup])
xticks(onset_baseline:0.1:onset_rest)

box on

% for  tick = onset_baseline+step : step : onset_rest-step
%     plot([tick, tick], [-y_limit,  -y_limit*0.9], '-k')
% end

%%%% Legend
load('z_labels_conditions.mat')

subplot(2,4,4)
hold on

x = 0:1;
y = ones(1, length(x));
xlim([-0.2, 1.4])
ylim([-7, 0])
scale = 0.3;

for cond = 1:6

    plot(x, -y*cond, 'color', color_line_plot(cond, :), 'LineWidth', 2.5)

    fill([x, fliplr(x)], ...
        [ -y*cond + y*scale,  ...
        fliplr(-y*cond - y*scale)], ...
        color_area_plot(cond, :), 'FaceAlpha', face_alpha, 'EdgeColor','none');
    text(1.05, -cond, [labels_conditions(cond, :), ' (GA ± SE) '])

end
yticklabels(-1:6)
title('   ')
subtitle('Legend', 'FontWeight', 'bold')

box off

fpath_save = fullfile(folder_save, sprintf('%s.png', save_title));
saveas(n_figure, fpath_save);

close all


%%
%%% Plot setting
n_figure = 1;
fig = figure(n_figure);
% fig.WindowState = "maximized";
fig.Position = dimensions; 
%sgtitle({title_label, fixed_condition}, 'FontSize', 12, 'FontWeight', 'bold', 'Color', '#5D5753');

save_title = 'avg_ch_all_comb_no_se';

subplot(2,4, [1 2 3, 5, 6,7])
conditions = 1:6;
for cond = conditions

    struct = load(fullfile(folder, sprintf('comb%d-ga_std_zscores.mat', cond)));

    %%%% Extract important times
    onset_baseline = struct.onset_baseline;
    onset_stim     = struct.onset_stim;
    onset_rest     = onset_stim +2;

    %%%% Extract GA across subject 
    data_avg = struct.data_ga_zscores;   
    
    %%%% Limit time axis PRE-STIM
    time_plot_indexes = find(struct.time >= onset_baseline & struct.time <= onset_stim + 2);
    time_plot = struct.time(time_plot_indexes);
    
    %%%% Extract GA for PRE_STIM
    data_avg = data_avg(ROI_channels_indexes, time_plot_indexes);
    
    %%%% Calculate GA average across channels and SE of GA across channels
    all_avg = mean(data_avg, 1);
    all_se = std(data_avg)/sqrt(n_ROI_channels);

    hold on
    plot(time_plot, all_avg * factor, ...
        'LineWidth', line_width, 'Color', color_line_plot(cond, :)')

end %%% end conditions
xline(onset_stim, '--',  'LineWidth', 0.5)

% xregion(start_region_sep, stop_region_sep, 'FaceColor','g', 'FaceAlpha', 0.2)
% 
% for x = [50, 150, 300]
%     xline(x*1e-3,'-', 'Color', 'b', 'LineWidth', 0.8)
%     text(x*1e-3, 0.75, sprintf('P%d', x), 'Color', 'b', 'FontSize', 4, 'HorizontalAlignment', 'center')
%     xregion((x-20)*1e-3, (x+20)*1e-3, 'FaceColor','b', 'FaceAlpha', 0.1)
% end
% 
% for x = 200
%     xline(x*1e-3, '-', 'Color', 'r', 'LineWidth', 0.8)
%     text(x*1e-3, -0.75, sprintf('N%d', x), 'Color', 'r', 'FontSize', 4, 'HorizontalAlignment', 'center')
%     xregion((x-20)*1e-3, (x+20)*1e-3, 'FaceColor','r', 'FaceAlpha', 0.1)
% end

text(0.65, 0.93,'STIM', 'Units', 'normalized', 'FontSize', 12, 'Color', '#636363', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
text(0.10, 0.93,'PRE ', 'Units', 'normalized', 'FontSize', 12, 'Color', '#636363', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');

ylabel(y_label)
xlabel(x_label)
ylim([y_limit_inf, y_limit_sup])

yticks([y_limit_inf,  0, y_limit_sup])
xticks(onset_baseline:0.1:onset_rest)

box on

title('   ')
subtitle('Grand average evolution', 'FontWeight', 'bold')

% for  tick = onset_baseline+step : step : onset_rest-step
%     plot([tick, tick], [-y_limit,  -y_limit*0.9], '-k')
% end

%%%% Legend
load('z_labels_conditions.mat')

subplot(2,4,4)
hold on

x = 0:1;
y = ones(1, length(x));
xlim([-0.2, 1.4])
ylim([-7, 0])
scale = 0.3;

for cond = 1:6

    plot(x, -y*cond, 'color', color_line_plot(cond, :), 'LineWidth', 2.5)
   
    text(1.05, -cond, [labels_conditions(cond, :), ' (GA)'])
end
yticklabels(-1:6)

title('   ')
subtitle('Legend', 'FontWeight', 'bold')

box off


fpath_save = fullfile(folder_save, sprintf('%s.png', save_title));
saveas(n_figure, fpath_save);

close all