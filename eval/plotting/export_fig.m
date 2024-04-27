function export_fig(fig, filepath)

    if ~verLessThan('matlab', '9.8')
        % PDF
        exportgraphics(fig, filepath, 'ContentType', 'vector');
        % EPS
        [folderpath,name,~] = fileparts(filepath);
        exportgraphics(fig, fullfile(folderpath,strcat(name,'.eps')), 'ContentType', 'vector');
    else
        % validFontNames = {'AvantGarde'; 'Bookman'; 'Courier'; 'Helvetica'; ...
        %             'Helvetica-Narrow'; 'NewCenturySchoolBook'; 'Palatino'; ...
        %             'Symbol'; 'Times'; 'ZapfChancery'; 'ZapfDingbats'};
        warning('off', 'MATLAB:print:FigureTooLargeForPage')
        print(fig, filepath, '-vector', '-dpdf');
        warning('on', 'MATLAB:print:FigureTooLargeForPage')
    end

end
