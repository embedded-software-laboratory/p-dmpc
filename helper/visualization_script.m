%% visualize lanelet
% [lanelets, adjacency_lanelets, semi_adjacency_lanelets, intersection_lanelets,...
%    lanelet_boundary, road_raw_data, lanelet_relationships] = get_road_data();
road_data = RoadData().get_road_data();

lanelet_boundary_poly = cellfun(@(c)[c{3}],road_data.lanelet_boundary);
figure()
plot(lanelet_boundary_poly)
hold on
box on
axis equal

xlabel('\fontsize{14}{0}$x$ [m]','Interpreter','LaTex');
ylabel('\fontsize{14}{0}$y$ [m]','Interpreter','LaTex');

xlim([0,4.5]);
ylim([0,4]);
daspect([1 1 1])