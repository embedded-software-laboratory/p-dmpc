%% visualize lanelet
% get road data
road_data = RoadData().get_road_data();

%% plot lanelets
figure()
plot_lanelets(road_data.lanelets,'Commonroad');
plotRoadSetup;

%% plot lanelet boundaries
figure()
lanelet_boundary_poly = cellfun(@(c)[c{3}],road_data.lanelet_boundary);
plot(lanelet_boundary_poly)
plotRoadSetup;

%% local function
function plotRoadSetup()
% Setup of plot()
    hold on
    box on
    axis equal
    
    xlabel('\fontsize{14}{0}$x$ [m]','Interpreter','LaTex');
    ylabel('\fontsize{14}{0}$y$ [m]','Interpreter','LaTex');
    
    xlim([0,4.5]);
    ylim([0,4]);
    daspect([1 1 1])
end