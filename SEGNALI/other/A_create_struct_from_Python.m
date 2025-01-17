clc, clear all, close all

n_subjects =   5
n_conditions = 6

save_mat = 1

time_correction = 0.1

path_python = 'epo-python';
path_save = 'epo-original'; funcCreateFolder(path_save)

clc

for cond = 1 : n_conditions
    disp("  ")

    for subj = 1 : n_subjects

        fprintf('cond%d - s00%d', cond, subj)
        disp("  ")

        % Load file
        struct = load(fullfile(path_python, sprintf('mne_comb%d_s00%d-epo.mat', cond, subj))); 

        data = permute(struct.data_mne(:, :, 1:end-1), [2, 3, 1]);

        % Pre-allocate cell array
        ch_names = cell(1, length(struct.ch_names_mne));
        subject_id = cell(1, 1);
        condition = cell(1, 1);

        % Remove last time sample
        time = struct.time(1:end-1) - time_correction;

        % Create the cell array
        for n = 1:1:length(struct.ch_names_mne)
            ch_names{n} = struct.ch_names_mne(n,:);
        end

        subject_id{1} = struct.subject_id_mne(:);
        condition{1} = struct.condition_mne(:);

        events_mne = struct.events_mne; 
        sfreq = struct.sfreq; 

        if save_mat
            % Save the new struct
            filepath = fullfile(path_save, sprintf('comb%d_s00%d-epo_original.mat', cond, subj));
            save(filepath, 'data','events_mne', 'sfreq', 'time', 'ch_names', 'subject_id', 'condition') ;
        end
    end %%% end subjects
end %%% end combinations

disp("  ")
disp('END')


%% Check permute

% ep_check = 4;
% ch_check = 2;
%
% Atest = squeeze(data_mne(ep_check, ch_check, :))';
% Btest = squeeze(data(ch_check, :, ep_check));
%
% figure
% plot(Atest-Btest, 'ob', 'MarkerFaceColor','blue'),
% legend('Difference')








