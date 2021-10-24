function plot_commonroad_lanelets(commonroad_lanelets)

Nlanelets = length(commonroad_lanelets{1});

hold on
for i = 1:Nlanelets
    plot(commonroad_lanelets{1}{i},commonroad_lanelets{2}{i},'color','k')
    plot(commonroad_lanelets{3}{i},commonroad_lanelets{4}{i},'color','k')
end


end