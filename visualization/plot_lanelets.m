function plot_lanelets(road_raw_data,scenario_name)
% PLOT_LANELETS     Plots the lanelet structure.

    for i = 1 : length(road_raw_data)

        switch scenario_name
            case 'Commonroad'
                color = '#848484';
            otherwise
                if i < 13
                    color = '#848484';
                else
                    if i < 17
                       color = '#E50000';
                    else
                       color = '#0080FF';
                    end
                end
        end
        plot_lanelet(road_raw_data(i),color);
        
    end

end

function plot_lanelet(road_raw_data, color)
            
            % left bound
            if strcmp(road_raw_data.rightBound.lineMarking,'dashed')
                lineStyle = '--';
            else
                lineStyle = '-';
            end
            line([road_raw_data.rightBound.point.x], [road_raw_data.rightBound.point.y], 'Color', color, 'LineWidth', 0.15, 'LineStyle', lineStyle);

            % right bound
            if strcmp(road_raw_data.leftBound.lineMarking,'dashed')
                lineStyle = '--';
            else
                lineStyle = '-';
            end
            line([road_raw_data.leftBound.point.x], [road_raw_data.leftBound.point.y], 'Color', color, 'LineWidth', 0.15, 'LineStyle', lineStyle);

end
