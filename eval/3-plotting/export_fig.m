function export_fig(fig, filepath)

    % PDF
    exportgraphics(fig, filepath, 'ContentType', 'vector');

    % EPS
    [folderpath, name, ~] = fileparts(filepath);
    exportgraphics(fig, fullfile(folderpath, strcat(name, '.eps')), 'ContentType', 'vector');

    % EMF
    if ispc
        exportgraphics(fig, fullfile(folderpath, strcat(name, '.emf')), 'ContentType', 'vector');
    end

end
