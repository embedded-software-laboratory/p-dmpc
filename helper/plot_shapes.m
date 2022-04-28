function plot_shapes(shapes)
%PLOT_REACHABLESETS Plot shapes (vehicles' predicted occupied areas or
%reachable sets) in a certain time steps (normally Hp time steps)
    
    Hp = size(shapes,2);
    CM = jet(Hp); % colormap
    for iVeh=1:size(shapes,1)
        for t=1:Hp
            plot([shapes{iVeh,t}(1,:) shapes{iVeh,t}(1,1)],[shapes{iVeh,t}(2,:) shapes{iVeh,t}(2,1)],"Color",CM(t,:)); % to plot a closed area
        end
    end

end