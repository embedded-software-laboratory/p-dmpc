function export_fig(fig, filepath)
    if ~verLessThan('matlab','9.8')
        exportgraphics(fig, filepath, 'ContentType','vector');
    else
        print(fig,filepath,'-depsc');
    end
end