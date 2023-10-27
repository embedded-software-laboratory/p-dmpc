function set_figure_properties(figHandle, export_fig_config)
    % SET_FIGURE_PROPERTIES     Set Properties used for figures based on the type of export.
    arguments
        figHandle (1, 1) matlab.ui.Figure;
        export_fig_config (1, 1) ExportFigConfig;
    end

    % background color
    set(figHandle, 'Color', 'w');
    % format
    set(figHandle, 'Units', export_fig_config.units);
    screenpos = get(figHandle, 'Position');
    set(figHandle ...
        , 'Position', [screenpos(1:2), export_fig_config.paperwidth, export_fig_config.paperheight] ... % px, py, w, h, of figure on screen
        , 'PaperSize', [export_fig_config.paperwidth, export_fig_config.paperheight] ... % px, py, w, h, of figure on print
    );

    colororder(rwth_color_order());

    % Workaround to keep desired width
    annotation('line', [0, 0], [0.49, 0.51] ...
        , 'Color', 'w' ...
        , 'LineWidth', 0.01 ...
    )
    annotation('line', [1, 1], [0.49, 0.51] ...
        , 'Color', 'w' ...
        , 'LineWidth', 0.01 ...
    )

    % beauty corrections
    allchildren = get(figHandle, 'Children'); % get handle figure

    for a = 1:length(allchildren)

        try % set title handle
            h_title = get(allchildren(a), 'Title');
            set(h_title, ...
                'FontWeight', 'normal', ...
                'FontSize', export_fig_config.fontsize + 1, ...
                'FontName', export_fig_config.fontname, ...
                'Interpreter', 'latex');
        catch
            % continue
        end

        try % redefine x- and y-labels
            h_xlabel = get(allchildren(a), 'xlabel');
            h_ylabel = get(allchildren(a), 'ylabel');
            set(h_xlabel, ...
                'FontSize', export_fig_config.fontsize, ...
                'FontName', export_fig_config.fontname, ...
                'Interpreter', 'latex')
            set(h_ylabel, ...
                'FontSize', export_fig_config.fontsize, ...
                'FontName', export_fig_config.fontname, ...
                'Interpreter', 'latex')
        catch
            % continue
        end

        try % redefine z-label
            h_zlabel = get(allchildren(a), 'zlabel');
            set(h_zlabel, ...
                'FontSize', export_fig_config.fontsize, ...
                'FontName', export_fig_config.fontname, ...
                'Interpreter', 'latex')
        catch
            % continue
        end

        % set axes
        try
            h_axes = get(allchildren(a), 'Axes');
            set(h_axes, ...
                'FontSize', export_fig_config.fontsize, ...
                'FontName', export_fig_config.fontname, ...
                'LineWidth', export_fig_config.linewidth, ...
                'Box', 'on');
        catch
            % continue
        end

        % set subplotaxes
        try
            set(allchildren(a) ...
                , 'FontSize', export_fig_config.fontsize ...
                , 'FontName', export_fig_config.fontname ...
                , 'LineWidth', export_fig_config.linewidth ...
                , 'Box', 'on' ...
            );
        catch
            % continue
        end

        % set legend
        if strcmpi(get(allchildren(a), 'Tag'), 'legend')
            h_legend = allchildren(a);
            set(h_legend, 'FontSize', export_fig_config.fontsize - 3)
            set(h_legend, ...
                'LineWidth', export_fig_config.linewidth, ...
                'FontName', export_fig_config.fontname, ...
                'Interpreter', 'latex', ...
                'Box', 'on' ...
            );
        end

        % Set graphic objects

        h_graphics = get(allchildren(a), 'Children');

        for h_graphic = h_graphics'

            try
                set(h_graphic ...
                    , 'LineWidth', export_fig_config.linewidth ...
                );
            catch
                % continue
            end

            try
                set(h_graphic ...
                    , 'MarkerSize', export_fig_config.markersize ...
                );
            catch
                % continue
            end

            try
                set(h_graphic, ...
                    'FontSize', export_fig_config.fontsize, ...
                    'FontName', export_fig_config.fontname);
            catch
                % continue
            end

            if strcmpi(get(h_graphic, 'Tag'), 'circle')
                coordinates = get(h_graphic, 'Position');
                % get possible size of text in radius
                uc = uicontrol('Style', 'text', 'Visible', 'Off', 'Units', 'normalized', ...
                    'FontSize', export_fig_config.fontsize, 'FontName', export_fig_config.fontname, 'String', '20');
                string_coordinates = get(uc, 'Extent');
                delete(uc);
                x_limits = xlim;
                y_limits = ylim;
                % compute new radius so that text fits into circle
                new_radius = max(string_coordinates(end - 1) * (x_limits(2) - x_limits(1)), ...
                    string_coordinates(end) * (y_limits(2) - y_limits(1))) * 0.8/2;
                % apply new radius if the circle needs to be larger
                old_radius = coordinates(end) / 2;

                if new_radius > old_radius
                    new_coordinates = [coordinates(1) + old_radius - new_radius, ...
                                           coordinates(2) + old_radius - new_radius, 2 * new_radius, 2 * new_radius];
                    set(h_graphic, 'Position', new_coordinates);
                end

            end

        end

    end

end
