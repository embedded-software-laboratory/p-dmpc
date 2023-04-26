function plot_obstacles(data, plot_options)
    % PLOT_OBSTACLES Plot obstacles.
    %
    % INPUT:
    %   data: could be two types of data
    %       1. cell array, where each cell contains a two-row matrix, the first
    %       and second rows being the x- and y-coordinates respectively.
    %       2. instance of the class "Scenario", where the properties
    %       "obstacles", "dynamic_obstacle_area" and
    %       "dynamic_obstacle_reachableSets" are the above mentioned cell
    %       array.
    %

    if nargin == 1
        plot_options = [];
    end

    if isempty(data)
        disp('Empty data!')
        return
    end

    if ismatrix(data) && isnumeric(data)
        plot_shape(data, plot_options)
    elseif strcmp(class(data), 'polyshape')
        plot(data, 'LineWidth', 1.0)
    elseif iscell(data)

        if isempty(data{1})
            disp('Empty data!')
            return
        end

        plot_cell_array_of_shapes(data, plot_options)
    elseif strcmp(class(data), 'Scenario')
        % if input is an instance of the class "Scenario"
        plot_cell_array_of_shapes(data.obstacles, plot_options)
        plot_cell_array_of_shapes(data.dynamic_obstacle_area, plot_options)
        plot_cell_array_of_shapes(data.dynamic_obstacle_reachableSets, plot_options)
        plot_cell_array_of_shapes(data.lanelet_crossing_areas, plot_options)
    else
        warning('Not supported data type.')
    end

end

%% local function
function plot_cell_array_of_shapes(shapes, plot_options)
    % Plot shapes contained in a cell array
    %     CM = jet(Hp); % colormap
    hold on

    if isempty(plot_options)
        is_empty = true;
        CM = hsv(size(shapes, 2));
    else
        is_empty = false;
    end

    for iVeh = 1:size(shapes, 1)

        for t = 1:size(shapes, 2)
            shape = shapes{iVeh, t};

            if is_empty
                plot_options = struct('LineWidth', 1.0, 'Color', CM(t, :));
            end

            if strcmp(class(shape), 'polyshape')
                plot(shape)
            else
                plot_shape(shape, plot_options)
            end

        end

    end

end

%% local function
function plot_shape(shape, plot_options)

    if ~isempty(plot_options)
        plot(shape(1, :), shape(2, :), plot_options);
    else
        plot(shape(1, :), shape(2, :), 'LineWidth', 1.0);
    end

end
