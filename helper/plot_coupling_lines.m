function plot_coupling_lines(directed_adjacency, iter, belonging_vector)
%UNTITLED2 Summary of this function goes here
    
    nVeh = length(directed_adjacency);

    if nargin == 2
        belonging_vector = [];
    end
    for v = 1:nVeh
        x = iter.x0(v,:);

        % plot directed coupling
        adjacent_vehicles = find(directed_adjacency(v,:) ~= 0);

        for adj_v = adjacent_vehicles
            adj_x = iter.x0(adj_v,:);
            % plot adjacency
            if ~isempty(belonging_vector) && belonging_vector(v) ~= belonging_vector(adj_v)
                % if there are parallel groups, couplings inside a group
                % will be shown in red lines, while couplings between
                % groups will be shown in blue dashed lines.
                quiver(x(1),x(2),adj_x(1)-x(1),adj_x(2)-x(2),'AutoScale','off','LineWidth',1,'Color','b','LineStyle','--')
            else
                quiver(x(1),x(2),adj_x(1)-x(1),adj_x(2)-x(2),'AutoScale','off','LineWidth',1,'Color','r','LineStyle','-');
            end
        end
    end

end