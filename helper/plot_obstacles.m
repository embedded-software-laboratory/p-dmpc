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

    if iscell(data)
        plot_shapes(data)
    elseif strcmp(class(data), 'Scenario')
        % if input is a instance of the class "Scenario"
        plot_shapes(data.obstacles)
        plot_shapes(data.dynamic_obstacle_area)
        plot_shapes(data.dynamic_obstacle_reachableSets)
    end

end


%% local function
function plot_shapes(shapes)
% PLOT_SHAPES Plot shapes (vehicles' predicted occupied areas or
% reachable sets) in a certain time steps (normally Hp time steps)
    
%     CM = jet(Hp); % colormap
    hold on
    for iVeh=1:size(shapes,1)
        for t=1:size(shapes,2)
            shape_type = class(shapes{iVeh,t});
            switch shape_type
                case 'polyshape'
                    plot(shapes{iVeh,t},'LineWidth',1.0) % use line width 1.0 if the plotted shapes should be deleted at the next iteration (see plotOnline.m)
                otherwise
%                     plot([shapes{iVeh,t}(1,:) shapes{iVeh,t}(1,1)],[shapes{iVeh,t}(2,:) shapes{iVeh,t}(2,1)],"Color",CM(t,:),'LineWidth',1.0); % to plot a closed area
                    plot([shapes{iVeh,t}(1,:) shapes{iVeh,t}(1,1)],[shapes{iVeh,t}(2,:) shapes{iVeh,t}(2,1)],'LineWidth',1.0); % to plot a closed area
            end
        end
    end

end

