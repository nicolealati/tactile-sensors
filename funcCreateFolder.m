function funcCreateFolder(fpath)

% Crea cartella (se non esiste)
if ~exist(fpath, 'dir')
    mkdir(fpath);
    disp("Created folder")
else
    disp("Folder already existing")
end

end

