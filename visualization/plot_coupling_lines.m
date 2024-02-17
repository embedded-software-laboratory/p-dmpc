function plot_coupling_lines( ...
        weighted_coupling, ...
        directed_coupling_sequential, ...
        x0, ...
        coupling_visu, ...
        optional ...
    )
    % PLOT_COUPLING_LINES This function visualizes the coupling between
    % each coupled pair.
    arguments
        weighted_coupling (:, :) double;
        directed_coupling_sequential (:, :) double;
        x0 (:, 3) double; % vehicle states
        % a struct to control the visualization of coupling lines
        coupling_visu (1, 1) struct;
        % a matrix whose value indicate whether the coupling is handled by virtual obstacles
        optional.is_virtual_obstacle (:, :) logical = false(size(weighted_coupling));
    end

    is_virtual_obstacle = optional.is_virtual_obstacle;

    % if the given matrix is adjacency matrix but not edge-weights matrix
    if all(weighted_coupling == 1 | weighted_coupling == 0, "all")
        coupling_visu.isShowValue = false;
    end

    nVeh = length(weighted_coupling);

    color_main = [0 0 0];
    color_minor = 0.4 * [1 1 1];
    head_size = coupling_visu.radius;

    for v = 1:nVeh
        x = x0(v, :);
        % plot directed coupling
        adjacent_vehicles = find(weighted_coupling(v, :));

        % find_couplings that are handly by virtual obstacles
        coupling_is_virtual_obstacle = find(is_virtual_obstacle(v, :));

        % plot couplings that are ignored by letting vehicles without
        % right-of-way not enter the lanelet crossing area (virtual_obstacle)


        if ~isempty(coupling_is_virtual_obstacle)

            for coupled_i = coupling_is_virtual_obstacle
                coupled_x = x0(coupled_i, :);
                % couplings that are ignored will be shown in grey solid lines
                plot_arrow(x', (coupled_x - x)', coupling_visu.radius, coupling_visu.LineWidth, color_minor, '-', head_size);
            end

        end

        for adj_v = adjacent_vehicles
            adj_x = x0(adj_v, :);

            % plot adjacency

            is_coupling_sequential = directed_coupling_sequential(v, adj_v);

            if ~is_coupling_sequential
                plot_arrow(x', (adj_x - x)', coupling_visu.radius, coupling_visu.LineWidth, color_main, ':', head_size);
            else
                plot_arrow(x', (adj_x - x)', coupling_visu.radius, coupling_visu.LineWidth, color_main, '-', head_size);
            end

            if coupling_visu.isShowValue
                % plot coupling weights
                text( ...
                    (x(1) + adj_x(1)) / 2, (x(2) + adj_x(2)) / 2 ...
                    , num2str(round(weighted_coupling(v, adj_v), 2)) ...
                    , FontSize = coupling_visu.FontSize ...
                    , LineWidth = coupling_visu.LineWidth ...
                    , Color = color_main ...
                    , Tag = "temporary" ...
                );
            end

        end

    end

end

%% local function to plot one arrow between vehicles
function [] = plot_arrow(x, u, radius, linewidth, color, linestyle, headsize)
    % Only use position.
    x = x(1:2);
    u = u(1:2);

    % Transform arrow size so that it starts and ends not in the vehicle middle, but at a circle around the center with given radius.
    l = sqrt(u(1) * u(1) + u(2) * u(2));
    x = x + radius * u / l;
    u = u - 2 * radius * u / l;
    l_shortened = sqrt(u(1) * u(1) + u(2) * u(2));

    % Transform direction to not contain arrow head.
    u_without_head = u * (1 - headsize / l_shortened);

    % Create triangle for arrow head from normal.
    n = 0.4 * headsize / l_shortened * [0 -1; 1 0] * u;
    head = [x + u, x + u_without_head + n, x + u_without_head - n, x + u];

    % Plot.
    line( ...
        [x(1), x(1) + u_without_head(1)], ...
        [x(2), x(2) + u_without_head(2)], ...
        Color = color, ...
        LineStyle = linestyle, ...
        LineWidth = linewidth, ...
        Tag = "temporary" ...
    );
    patch( ...
        head(1, :), head(2, :), ...
        color, ...
        EdgeColor = color, ...
        LineWidth = linewidth, ...
        Tag = "temporary" ...
    );
end
