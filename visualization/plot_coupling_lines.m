function plot_coupling_lines(M, x0, varargin)
    % PLOT_COUPLING_LINES This function visualizes the coupling between
    % each coupled pair. Red lines for coupling of two vehicles inside the same
    % group, while blue dashed lines are used if they are in the different
    % groups.
    %
    % INPUT:
    %   M: directed adjacency matrix or edge-weights matrix
    %
    %   x0: states
    %
    %   belonging_vector (optional input, use [] as input if not fit): a column
    %   vector whose value indicate which group each vehicle belongs to.For
    %   example,"belonging_vector = [1;2;2;1;3]" means the 1st subgraph =
    %   {1,4}, the 2nd subgraph = {2,3} and the 3rd subgraph = {5}
    %
    %   ShowWeights: logical, is show value of weights
    %

    % Process optional input and Name-Value pair options
    [M, x0, belonging_vector, coupling_info, coupling_visu] = parse_inputs(M, x0, varargin{:});

    % if the given matrix is adjacency matrix but not edge-weights matrix
    if all(M == 1 | M == 0, "all")
        coupling_visu.isShowValue = false;
    end

    nVeh = length(M);

    color_main = [0 0 0];
    color_minor = 0.4 * [1 1 1];

    for v = 1:nVeh
        x = x0(v, :);
        % plot directed coupling
        adjacent_vehicles = find(M(v, :) ~= 0);

        % plot couplings that are ignored by letting vehicles without
        % right-of-way not enter the lanelet crossing area
        if ~isempty(coupling_info)
            find_self = [coupling_info.veh_with_ROW] == v;
            find_ignored = [coupling_info.is_ignored] == true;
            find_targrt = all([find_self; find_ignored], 1);

            if any(find_targrt)
                all_adjacent_vehicles = [coupling_info(find_targrt).veh_without_ROW];
                coupling_ignored = setdiff(all_adjacent_vehicles, adjacent_vehicles);

                for i_ignored = coupling_ignored(:)'
                    ignored_x = x0(i_ignored, :);
                    MaxHeadSize = 0.7 * norm([ignored_x(1) - x(1), ignored_x(2) - x(2)]); % to keep the arrow size
                    % couplings that are ignored will be shown in grey solid lines
                    plot_arrow(x', (ignored_x - x)', coupling_visu.radius, coupling_visu.LineWidth, color_minor, '-', MaxHeadSize);
                end

            end

        end

        for adj_v = adjacent_vehicles
            adj_x = x0(adj_v, :);

            % plot adjacency
            MaxHeadSize = 0.7 * norm([adj_x(1) - x(1), adj_x(2) - x(2)]); % to keep the arrow size

            is_coupling_parallel = (~isempty(belonging_vector) && belonging_vector(v) ~= belonging_vector(adj_v));

            if is_coupling_parallel
                plot_arrow(x', (adj_x - x)', coupling_visu.radius, coupling_visu.LineWidth, color_main, ':', MaxHeadSize);
            else
                plot_arrow(x', (adj_x - x)', coupling_visu.radius, coupling_visu.LineWidth, color_main, '-', MaxHeadSize);
            end

            if coupling_visu.isShowValue
                % plot coupling weights
                text((x(1) + adj_x(1)) / 2, (x(2) + adj_x(2)) / 2, ...
                    num2str(round(M(v, adj_v), 2)), 'FontSize', coupling_visu.FontSize, 'LineWidth', coupling_visu.LineWidth, 'Color', color_main);
            end

        end

    end

end

%% local function
function [M, x0, belonging_vector, coupling_info, coupling_visu] = parse_inputs(M, x0, varargin)
    % Process optional input and Name-Value pair options

    default_belonging_vector = ones(1, length(M)); % default all vehicles are in the same group
    default_coupling_info = [];
    default_coupling_visu = struct('FontSize', 12, 'LineWidth', 1, 'isShowLine', false, 'isShowValue', false);

    p = inputParser;
    addRequired(p, 'M', @(x) ismatrix(x) && (isnumeric(x) || islogical(x))); % must be numerical matrix
    addRequired(p, 'x0', @(x) isstruct(x) || ismatrix(x) && (isnumeric(x))); % must be numerical matrix
    addOptional(p, 'belonging_vector', default_belonging_vector, @(x) (isnumeric(x) && isvector(x)) || isempty(x)); % must be numerical vector or empty
    addOptional(p, 'coupling_info', default_coupling_info, @(x) isstruct(x) || isempty(x)); % must be struct or empty
    addOptional(p, 'coupling_visu', default_coupling_visu, @(x) isstruct(x) || isempty(x)); % must be struct or empty

    parse(p, M, x0, varargin{:}); % start parsing

    % get parsed inputs
    M = p.Results.M;
    x0 = p.Results.x0;
    belonging_vector = p.Results.belonging_vector;
    coupling_info = p.Results.coupling_info;
    coupling_visu = p.Results.coupling_visu;
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

    % Transform direction to not contain arrow head.
    headsize = min(headsize, 0.07);
    u_without_head = u * (1 - headsize / l);

    % Create triangle for arrow head from normal.
    n = 0.4 * headsize / l * [0 -1; 1 0] * u;
    head = [x + u, x + u_without_head + n, x + u_without_head - n, x + u];

    % Plot.
    line([x(1), x(1) + u_without_head(1)], [x(2), x(2) + u_without_head(2)], 'Color', color, 'LineStyle', linestyle, 'LineWidth', linewidth);
    patch(head(1, :), head(2, :), color, 'EdgeColor', color, 'LineWidth', linewidth);
end
