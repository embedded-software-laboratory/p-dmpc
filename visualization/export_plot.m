function export_plot(function_name, varargin, optional)

    arguments
        % function handle of plotting function
        function_name (1, 1) function_handle = @()[]
    end

    arguments (Repeating)
        % arguments for plotting function
        varargin (1, :)
    end

    arguments
        % optional arguments for export process
        optional.fig (1, 1) matlab.ui.Figure = figure("Visible", "on");
        optional.export_fig_cfg (1, 1) ExportFigConfig = ExportFigConfig.paper();
        optional.file_path (1, :) char = 'plot.pdf'
    end

    % call plotting function with given arguments
    function_name(varargin{:}, fig = optional.fig);

    set_figure_properties(optional.fig, optional.export_fig_cfg)

    export_fig(optional.fig, optional.file_path);
    if (~optional.fig.Visible); close(optional.fig); end
end
