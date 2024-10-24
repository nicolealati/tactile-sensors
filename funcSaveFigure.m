function funcSaveFigure(save_fig, n_figure, folder_save, name_figure, funcCreateFolder)

fpath_save = fullfile(folder_save, name_figure); disp(fpath_save)
    
if save_fig

    % Crea cartella e corrispondente percorso
    funcCreateFolder(folder_save)
    fpath_save = fullfile(folder_save, name_figure); disp(fpath_save)
    
    % Salva figura
    saveas(n_figure, fpath_save);
    disp('saved figure')
    
else
    disp('Not saved figure')
end

end

