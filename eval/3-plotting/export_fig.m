function export_fig(fig, filepath, optional)

    arguments
        fig (1, 1) matlab.ui.Figure;
        filepath (1, 1) string;
        optional.is_vector_graphic (1, 1) logical = true;
    end

    if optional.is_vector_graphic
        % PDF
        exportgraphics(fig, filepath, 'ContentType', 'vector');

        % EPS
        [folderpath, name, ~] = fileparts(filepath);
        exportgraphics(fig, fullfile(folderpath, strcat(name, '.eps')), 'ContentType', 'vector');

        % EMF
        if ispc
            exportgraphics(fig, fullfile(folderpath, strcat(name, '.emf')), 'ContentType', 'vector');
        end

    else
        % PNG
        exportgraphics(fig, filepath, 'ContentType', 'image');
    end

end
