function plot_obstacles(data)
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
    
        if ismatrix(data) && isnumeric(data)
            plot_shape(data)
        elseif strcmp(class(data),'polyshape')
            plot(data,'LineWidth',1.0)
        elseif iscell(data)
            plot_cell_array_of_shapes(data)
        elseif strcmp(class(data), 'Scenario')
            % if input is an instance of the class "Scenario"
            plot_cell_array_of_shapes(data.obstacles)
            plot_cell_array_of_shapes(data.dynamic_obstacle_area)
            plot_cell_array_of_shapes(data.dynamic_obstacle_reachableSets)
            plot_cell_array_of_shapes(data.lanelet_intersecting_areas)
        else
            warning('Not supported data type.')
        end
    
    end
    
    
    %% local function
    function plot_cell_array_of_shapes(shapes)
    % Plot shapes contained in a cell array
    %     CM = jet(Hp); % colormap
        hold on
        for iVeh=1:size(shapes,1)
            for t=1:size(shapes,2)
                shape = shapes{iVeh,t};
                if strcmp(class(shape),'polyshape')
                    plot(shape)
                else
                    plot_shape(shape)
                end
            end
        end
    end
    
    
    %% local function
    function plot_shape(shape)
        plot(shape(1,:),shape(2,:),'LineWidth',1.0);
    end
    
    
