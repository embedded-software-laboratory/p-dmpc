function plot_lanelets(lanelets)
    % PLOT_LANELETS     Plots the lanelet structure.
    color = '#848484';

    for i = 1:length(lanelets)
        plot_lanelet(lanelets(i), color);
    end

end

function plot_lanelet(lanelet, color)
    % left bound
    if strcmp(lanelet.rightBound.lineMarking, 'dashed')
        lineStyle = '--';
    else
        lineStyle = '-';
    end

    line([lanelet.rightBound.point.x], [lanelet.rightBound.point.y], 'Color', color, 'LineWidth', 0.15, 'LineStyle', lineStyle);

    % right bound
    if strcmp(lanelet.leftBound.lineMarking, 'dashed')
        lineStyle = '--';
    else
        lineStyle = '-';
    end

    line([lanelet.leftBound.point.x], [lanelet.leftBound.point.y], 'Color', color, 'LineWidth', 0.15, 'LineStyle', lineStyle);
end
