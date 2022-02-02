function plot_lanelets(lanelets)
% PLOT_LANELETS     Plots the lanelet structure.

    for i = 1 : length(lanelets)
        
        lanelet = lanelets{i};
        

        if i < 13
            color = '#848484';
        else
            if i < 17
%                color = '#E50000';
                color = '#848484';
            else
%                color = '#0080FF';
                color = '#848484';
            end
        end
        plot_lanelet(lanelet,color);
        
    end

end

function plot_lanelet(lanelet, color)
            
            line(lanelet(:,LaneletInfo.rx), lanelet(:,LaneletInfo.ry), 'Color', color);
            line(lanelet(:,LaneletInfo.lx), lanelet(:,LaneletInfo.ly), 'Color', color);
            line(lanelet(:,LaneletInfo.cx), lanelet(:,LaneletInfo.cy), 'Color', color,'linestyle', '--'); 

end
