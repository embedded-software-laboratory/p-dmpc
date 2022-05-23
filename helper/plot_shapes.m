function plot_shapes(shapes)
% PLOT_SHAPES Plot shapes (vehicles' predicted occupied areas or
% reachable sets) in a certain time steps (normally Hp time steps)
    
    Hp = size(shapes,2);
%     CM = jet(Hp); % colormap
    hold on
    for iVeh=1:size(shapes,1)
        for t=1:Hp
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