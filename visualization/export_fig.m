function export_fig(fig, filepath)
    if ~verLessThan('matlab','9.8')
        exportgraphics(fig, filepath, 'ContentType','vector');
    else
        % validFontNames = {'AvantGarde'; 'Bookman'; 'Courier'; 'Helvetica'; ...
%             'Helvetica-Narrow'; 'NewCenturySchoolBook'; 'Palatino'; ...
%             'Symbol'; 'Times'; 'ZapfChancery'; 'ZapfDingbats'};
        warning('off','MATLAB:print:FigureTooLargeForPage')
        print(fig,filepath,'-vector','-dpdf');
        warning('on','MATLAB:print:FigureTooLargeForPage')
    end
end