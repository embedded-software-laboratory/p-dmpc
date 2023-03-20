%% print vehicle initial positions to fig
nVeh = 40;
vehs = {};
vehicle_ids = 1:nVeh;

road_data = RoadData().get_road_data;
for iveh = 1:nVeh
    
    veh = Vehicle();

    [ref_path,loop_idx] = generate_ref_path_loop(vehicle_ids(iveh), road_data.lanelets);% function to generate refpath based on CPM Lab road geometry
    
    veh.ref_path_loop_idx = loop_idx;
    refPath = ref_path.path;

    if loop_idx == 1
        % for the first loop of path, vehicle's initial position is not
        % exactly at the starting of a lanelet but some points after to
        % avoid that a vehicle is parallel to another vehicle at the begining. This is
        % problematic when the maximum computation level is only 1, because
        % on of them will not able to find a feasible trajectory at the
        % first time step.
        start_point_idx = 5;
        veh.x_start = refPath(start_point_idx,1);
        veh.y_start = refPath(start_point_idx,2);
        veh.x_goal = [refPath(start_point_idx+1:end,1);refPath(2:start_point_idx-1,1);veh.x_start]; % add the starting point to the end to close the loop
        veh.y_goal = [refPath(start_point_idx+1:end,2);refPath(2:start_point_idx-1,2);veh.y_start]; % add the starting point to the end to close the loop
        
        veh.referenceTrajectory = [veh.x_start veh.y_start
                                   veh.x_goal  veh.y_goal];
        veh.lanelets_index = ref_path.lanelets_index;
        veh.points_index = ref_path.points_index;
        yaw = calculate_yaw(refPath);
        veh.yaw_start = yaw(start_point_idx);
        veh.yaw_goal = [yaw(start_point_idx+1:end);yaw(1:start_point_idx-1)];
    else
        veh.x_start = refPath(1,1);
        veh.y_start = refPath(1,2);
        veh.x_goal = refPath(2:end,1);
        veh.y_goal = refPath(2:end,2);
        
        veh.referenceTrajectory = [veh.x_start veh.y_start
                                   veh.x_goal  veh.y_goal];
        veh.lanelets_index = ref_path.lanelets_index;
        veh.points_index = ref_path.points_index;

        yaw = calculate_yaw(refPath);
        veh.yaw_start = yaw(1);
        veh.yaw_goal = yaw(2:end);
    end
    vehs{iveh} = veh;
end

loops = unique(cellfun(@(c) c.ref_path_loop_idx, vehs));
num_loops = numel(unique(cellfun(@(c) c.ref_path_loop_idx, vehs)));

file_name = 'vehInitialPositions.mat';

[file_path,~,~] = fileparts(mfilename('fullpath')); % get the path of the current file
idcs = strfind(file_path,filesep); % find all positions of '/'
two_folders_up = file_path(1:idcs(end-1)-1); % two folders up
full_path = fullfile(two_folders_up,'results','Commonroad_RHC-Parl',file_name);

set(0,'DefaultTextFontname', 'Times New Roman');
set(0,'DefaultAxesFontName', 'Times New Roman');

set(0,'DefaultTextFontsize', 7)
set(0,'DefaultAxesFontsize', 7)

fig_x = 6.4;     fig_y = 6.4; % [cm]
x_margin = 0;   y_margin = 0; 
fig_x_position = fig_x - 2*x_margin;
fig_y_position = fig_y - 2*y_margin;

file_name = 'vehInitialPositions';

fig = figure('Name',file_name);
set(fig, 'Units','centimeters', 'Position',[0 0 fig_x_position fig_y_position]/2)
set(fig, 'PaperUnits','centimeters','PaperSize',[fig_x fig_y],'PaperOrientation','portrait',...
    'PaperPosition', [x_margin y_margin fig_x_position fig_y_position])
t_fig = tiledlayout(1,1,'Padding','compact','TileSpacing','compact');

step = 1;
tick_now = 1;

visu.isShowVehID = true;             
visu.isShowPriority = false;         
visu.isShowCoupling = false;          
visu.isShowWeight = false;           
visu.isShowHotkeyDescription = false;
nexttile

vehColor = hsv(num_loops);
vehColor(3,1) = 0.75;
hold on
box on
axis equal
xlim([0 4.5]);
ylim([0 4.0]);
daspect([1 1 1])

plot_lanelets(road_data.road_raw_data.lanelet,'Commonroad');

xlabel('$x [m]$','Interpreter','latex');
ylabel('$y [m]$','Interpreter','latex');

% Reference path
for iLoop = 1:num_loops
    find_veh = find(cellfun(@(c) ismember(iLoop,c.ref_path_loop_idx), vehs));
    plot(vehs{find_veh(1)}.referenceTrajectory(:,1),vehs{find_veh(1)}.referenceTrajectory(:,2),'LineWidth',0.3,'Color',vehColor(iLoop,:),'LineStyle','--')
end

% Vehicle rectangles
for v=1:nVeh
    veh = vehs{v};
    x = [veh.x_start,veh.y_start,veh.yaw_start];
    vehiclePolygon = transformedRectangle(x(1),x(2),x(3), veh.Length,veh.Width);
    patch(   vehiclePolygon(1,:)...
            ,vehiclePolygon(2,:)...
            ,vehColor(veh.ref_path_loop_idx,:)...
            ,'LineWidth',0.2 ...
    );
end

% IDs, priorities
for v=1:nVeh
    veh = vehs{v};
    x = [veh.x_start,veh.y_start,veh.yaw_start];
    % plot the vehicle index
    if visu.isShowVehID
        text(x(1)+0.1,x(2)+0.1,num2str(v), 'LineWidth',0.2,'Color','k');
    end
end

% save fig
EvaluationParl.save_fig(fig,file_name)

%%
% prepare simulation options
options = OptionsMain;
options.environment = Environment.Simulation;
options.customResultName = '';
options.scenario_name = 'Commonroad';
options.trim_set = 9;
options.Hp = 5;
options.dt = 0.2;
options.T_end = 20;
options.isPB = true;
options.isAllowInheritROW = true;
options.max_num_CLs = 3;
options.strategy_consider_veh_without_ROW = '3';
options.strategy_enter_lanelet_crossing_area = '4';
options.isSaveResult = true;
options.visu = [false,false];
options.is_eval = false;
options.visualize_reachable_set = false;
options.priority = 'STAC_priority';
options.amount = 20;

% run simulation
if exist('scenario','var')
    [~,~,~] = main(options,scenario);
else
    [~,~,~] = main(options);
end
e_test = EvaluationParl(options);
%% Motion primitives
options = OptionsMain;
options.scenario_name = 'Commonroad';
options.trim_set = 9;
options.Hp = 5;
options.amount = 20;
options.isPB = true;
options.dt = 0.2;
options.recursive_feasibility = true;

veh = Vehicle;
model = BicycleModel(veh.Lf,veh.Lr);
mpa = MotionPrimitiveAutomaton(model,options);

set(0,'DefaultTextFontname', 'Times New Roman');
set(0,'DefaultAxesFontName', 'Times New Roman');

set(0,'DefaultTextFontsize',11)
set(0,'DefaultAxesFontsize',11)

options_trim_cur = struct('LineWidth',0.5,'Color',[0.4660 0.6740 0.1880],'LineStyle','-');
options_trim_cur_local = struct('LineWidth',0.5,'Color',[0.4660 0.6740 0.1880],'LineStyle','--');
options_trim_next = struct('LineWidth',0.5,'Color',[0 0.4470 0.7410],'LineStyle','-');
options_trim_next_local = struct('LineWidth',0.5,'Color',[0 0.4470 0.7410],'LineStyle','--');
options_transition = struct('LineWidth',0.5,'Color',[0.6350 0.0780 0.1840],'LineStyle','-');
options_transition_local = struct('LineWidth',0.5,'Color',[0.6350 0.0780 0.1840],'LineStyle','--');
options_motion_primitive = struct('LineWidth',1,'Color',[0.4940 0.1840 0.5560],'LineStyle','-');
options_motion_primitive_local = struct('LineWidth',1,'Color',[0.4940 0.1840 0.5560],'LineStyle',':');
fig_x = 16;     fig_y = 5.5; % [cm]
x_margin = 0;   y_margin = 0; 
fig_x_position = fig_x - 2*x_margin;
fig_y_position = fig_y - 2*y_margin;

file_name = 'motionPrimitives';
fig = figure('Name',file_name);
set(fig, 'Units','centimeters', 'Position',[0 0 fig_x_position fig_y_position]/2)
set(fig, 'PaperUnits','centimeters','PaperSize',[fig_x fig_y],'PaperOrientation','portrait',...
    'PaperPosition', [x_margin y_margin fig_x_position fig_y_position])

t_fig = tiledlayout(1,1,'Padding','loose','TileSpacing','loose');

xlabel('$x\:[m]$','Interpreter','latex');
ylabel('$y\:[m]$','Interpreter','latex');

trim_initial = 8;
trim_next = 9;
veh = Vehicle;
m = mpa.maneuvers{trim_initial,trim_next};

offset = 0.01;

area_initial_local = transformedRectangle(0,0,0,veh.Length,veh.Width);
area_initial_local = [area_initial_local,area_initial_local(:,1)]; % close shape
area_initial_local_with_offset = transformedRectangle(0,0,0,veh.Length+2*offset,veh.Width+2*offset);
area_initial_local_with_offset = [area_initial_local_with_offset,area_initial_local_with_offset(:,1)]; % close shape
area_next_local = transformedRectangle(m.dx,m.dy,m.dyaw, veh.Length,veh.Width);
area_next_local = [area_next_local,area_next_local(:,1)]; % close shape
area_next_local_with_offset = transformedRectangle(m.dx,m.dy,m.dyaw, veh.Length+2*offset,veh.Width+2*offset);
area_next_local_with_offset = [area_next_local_with_offset,area_next_local_with_offset(:,1)]; % close shape

hold on
grid on
axis equal
box on
clear p
arrow_length = 0.2;
quiver(0,0,0.2,0,'AutoScale','off','MaxHeadSize',0.5,'LineWidth',0.5,'Color',[0.4660 0.6740 0.1880],'LineStyle','--');
quiver(m.dx,m.dy,arrow_length*cos(m.dyaw),arrow_length*sin(m.dyaw),'AutoScale','off','MaxHeadSize',0.5,'LineWidth',0.5,'Color',[0 0.4470 0.7410],'LineStyle','--');

p(1) = plot(area_initial_local(1,:),area_initial_local(2,:),options_trim_cur_local);
p(2) = plot(area_next_local(1,:),area_next_local(2,:),options_trim_next_local);
p(3) = plot(m.xs,m.ys,options_motion_primitive_local);
p(4) = plot(m.area(1,:),m.area(2,:),options_transition_local);
plot(m.area_without_offset(1,:),m.area_without_offset(2,:),options_transition_local);

initial_states = [0.6,-0.2,pi/4];
area_initial_global = transformedRectangle(initial_states(1),initial_states(2),initial_states(3),veh.Length,veh.Width);
area_initial_global = [area_initial_global,area_initial_global(:,1)]; % close shape
[area_next_global_x,area_next_global_y] = translate_global(initial_states(3),initial_states(1),initial_states(2),area_next_local(1,:),area_next_local(2,:));
[MP_global_x,MP_global_y] = translate_global(initial_states(3),initial_states(1),initial_states(2),m.xs,m.ys);
MP_global_yaw = m.yaws + initial_states(3);
quiver(initial_states(1),initial_states(2),0.2*cos(initial_states(3)),0.2*sin(initial_states(3)),'AutoScale','off','MaxHeadSize',0.5,'LineWidth',0.5,'Color',[0.4660 0.6740 0.1880]);
quiver(MP_global_x(end),MP_global_y(end),0.2*cos(MP_global_yaw(end)),0.2*sin(MP_global_yaw(end)),'AutoScale','off','MaxHeadSize',0.5,'LineWidth',0.5,'Color',[0 0.4470 0.7410]);
p(5) = plot(area_initial_global(1,:),area_initial_global(2,:),options_trim_cur);
p(6) = plot(area_next_global_x,area_next_global_y,options_trim_next);
[area_trans_x, area_trans_y] = translate_global(initial_states(3),initial_states(1),initial_states(2),m.area(1,:),m.area(2,:));
area_trans = [area_trans_x;area_trans_y];
p(7) = plot(MP_global_x,MP_global_y,options_motion_primitive);
p(8) = plot(area_trans_x,area_trans_y,options_transition);

legend(p,{'Vehicle at trim 8 (local)','Vehicle at trim 9 (local)','Motion primitive from trim 8 to 9','Occupied set of the motion primitive', ...
    'Vehicle at trim 8 (global)','Vehicle at trim 9 (global)','Translated motion primitive','Translated occupied set'},'Location','westoutside','Interpreter','latex')

xlim([-0.2 0.9])
ylim([-0.4 0.2])
EvaluationParl.save_fig(fig,file_name)
%% visualize local reachable sets
options = OptionsMain;
options.scenario_name = 'Commonroad';
options.trim_set = 13;
options.Hp = 4;
options.amount = 20;
options.isPB = true;
options.dt = 0.2;
options.recursive_feasibility = true;
options.is_use_dynamic_programming = false;

veh = Vehicle;
model = BicycleModel(veh.Lf,veh.Lr);
[mpa,trimsInfo] = MotionPrimitiveAutomaton(model,options);

set(0,'DefaultTextFontname', 'Times New Roman');
set(0,'DefaultAxesFontName', 'Times New Roman');

set(0,'DefaultTextFontsize',11)
set(0,'DefaultAxesFontsize',11)

options_RS = {};
options_RS{1} = struct('LineWidth',0.5,'Color','k','LineStyle','-');
options_RS{2} = struct('LineWidth',0.5,'Color','k','LineStyle','--');
options_RS{3} = struct('LineWidth',0.5,'Color','k','LineStyle','-.');
options_RS{4} = struct('LineWidth',0.5,'Color','k','LineStyle',':');

% color map
CM = {[0.6350 0.0780 0.1840],[0.9290 0.6940 0.1250],[0 0.4470 0.7410],[0.4940 0.1840 0.5560]};

fig_x = 14;     fig_y = 7.5; % [cm]
x_margin = 0;   y_margin = 0; 
fig_x_position = fig_x - 2*x_margin;
fig_y_position = fig_y - 2*y_margin;

file_name = 'localReachableSets';
fig = figure('Name',file_name);
set(fig, 'Units','centimeters', 'Position',[0 0 fig_x_position fig_y_position]/2)
set(fig, 'PaperUnits','centimeters','PaperSize',[fig_x fig_y],'PaperOrientation','portrait',...
    'PaperPosition', [x_margin y_margin fig_x_position fig_y_position])

t_fig = tiledlayout(1,1,'Padding','loose','TileSpacing','loose');

hold on
grid on
axis equal
box on

xlabel('$x\:[m]$','Interpreter','latex');
ylabel('$y\:[m]$','Interpreter','latex');


Hp_half = ceil(options.Hp/2); % for time step greater then half of the prediction horizon, dynamic programming is used to save computation time

trim_root = 1;
scatter_size = 8;
scatter(0,0,scatter_size,CM{trim_root},'filled')
n_trims = numel(mpa.trims);

% plot trajectories (brute-force)
for t = Hp_half+1:options.Hp
    maneuvers = trimsInfo(trim_root,t).maneuvers;
    for iM = 1:length(maneuvers)
        m = maneuvers{iM};
        childTrim = trimsInfo(trim_root,t).childTrims(iM);
        plot(m.xs,m.ys,'LineWidth',0.2,'LineStyle','-','Color',[0 0 0 0.1])
    end
end

% plot trajectories (dynamic programming)
for t = 1:Hp_half
    maneuvers = trimsInfo(trim_root,t).maneuvers;
    for iM = 1:length(maneuvers)
        m = maneuvers{iM};
        childTrim = trimsInfo(trim_root,t).childTrims(iM);
        plot(m.xs,m.ys,'LineWidth',0.2,'LineStyle','-','Color','k')
    end
end
clear sc;
clear p_RS_local
% plot nodes
for t = 1:Hp_half
    maneuvers = trimsInfo(trim_root,t).maneuvers;
    for iM = 1:length(maneuvers)
        m = maneuvers{iM};
        childTrim = trimsInfo(trim_root,t).childTrims(iM);
        sc(childTrim) = scatter(m.xs(end),m.ys(end),scatter_size,CM{childTrim},'filled');
    end
    [RS_local_x,RS_local_y] = boundary(mpa.local_reachable_sets{trim_root,t});
    p_RS_local(t) = plot(RS_local_x,RS_local_y,options_RS{t});
end

for t = Hp_half+1:options.Hp
    % brute-force
    maneuvers = trimsInfo(trim_root,t).maneuvers;
    for iM = 1:length(maneuvers)
        m = maneuvers{iM};
        childTrim = trimsInfo(trim_root,t).childTrims(iM);
        scatter(m.xs(end),m.ys(end),scatter_size,CM{childTrim},'filled','MarkerFaceAlpha',0.075);
    end
    % dynamic programming
    [RS_local_x,RS_local_y] = boundary(mpa.local_reachable_sets{trim_root,t});
    p_RS_local(t) = plot(RS_local_x,RS_local_y,options_RS{t});
end
legend([sc,p_RS_local],{'Node with trim 1','Node with trim 2','Node with trim 3','Node with trim 4', ...
    'Reachable set at time step 1','Reachable set at time step 2','Reachable set at time step 3','Reachable set at time step 4'},'Location','east')
% legend({})
ymax = 0.5;
xlim([-0.2 1.8])
ylim([-ymax ymax])
yticks(-ymax:0.2:ymax)
EvaluationParl.save_fig(fig,file_name)

%% translate and rotate the local reachable sets
options = OptionsMain;
options.scenario_name = 'Commonroad';
options.trim_set = 13;
options.Hp = 4;
options.amount = 20;
options.isPB = true;
options.dt = 0.2;
options.recursive_feasibility = true;
options.is_use_dynamic_programming = false;

veh = Vehicle;
model = BicycleModel(veh.Lf,veh.Lr);
[mpa,trimsInfo] = MotionPrimitiveAutomaton(model,options);

set(0,'DefaultTextFontname', 'Times New Roman');
set(0,'DefaultAxesFontName', 'Times New Roman');

set(0,'DefaultTextFontsize',11)
set(0,'DefaultAxesFontsize',11)
transparent = 0.3;
options_veh_area_local = struct('LineWidth',0.5,'Color',[0 0.4470 0.7410 transparent],'LineStyle','-');
options_veh_area_global = struct('LineWidth',0.5,'Color',[0 0.4470 0.7410],'LineStyle','-');
options_RS_local = {};
options_RS_local{1} = struct('LineWidth',0.5,'Color',[0.75 0.75 0.75],'LineStyle','-');
options_RS_local{2} = struct('LineWidth',0.5,'Color',[0.75 0.75 0.75],'LineStyle','--');
options_RS_local{3} = struct('LineWidth',0.5,'Color',[0.75 0.75 0.75],'LineStyle','-.');
options_RS_local{4} = struct('LineWidth',0.5,'Color',[0.75 0.75 0.75],'LineStyle',':');
options_RS_global = {};
options_RS_global{1} = struct('LineWidth',0.5,'Color',[0 0 0],'LineStyle','-');
options_RS_global{2} = struct('LineWidth',0.5,'Color',[0 0 0],'LineStyle','--');
options_RS_global{3} = struct('LineWidth',0.5,'Color',[0 0 0],'LineStyle','-.');
options_RS_global{4} = struct('LineWidth',0.5,'Color',[0 0 0],'LineStyle',':');

fig_x = 14;     fig_y = 7.5; % [cm]
x_margin = 0;   y_margin = 0; 
fig_x_position = fig_x - 2*x_margin;
fig_y_position = fig_y - 2*y_margin;

file_name = 'globalReachableSets';
fig = figure('Name',file_name);
set(fig, 'Units','centimeters', 'Position',[0 0 fig_x_position fig_y_position]/2)
set(fig, 'PaperUnits','centimeters','PaperSize',[fig_x fig_y],'PaperOrientation','portrait',...
    'PaperPosition', [x_margin y_margin fig_x_position fig_y_position])

t_fig = tiledlayout(1,1,'Padding','tight','TileSpacing','none');

hold on
grid on
axis equal
box on

xlabel('$x\:[m]$','Interpreter','latex');
ylabel('$y\:[m]$','Interpreter','latex');

x0 = [1,-0.4,pi/6];
veh_area_initial_local = transformedRectangle(0,0,0,veh.Length,veh.Width);
veh_area_initial_local = [veh_area_initial_local,veh_area_initial_local(:,1)]; % close shape
veh_area_initial_global = transformedRectangle(x0(1),x0(2),x0(3),veh.Length,veh.Width);
veh_area_initial_global = [veh_area_initial_global,veh_area_initial_global(:,1)]; % close shape
clear p
p(1) = plot(veh_area_initial_local(1,:),veh_area_initial_local(2,:),options_veh_area_local);
p(2) = plot(veh_area_initial_global(1,:),veh_area_initial_global(2,:),options_veh_area_global);
arrow_length = 0.18;
quiver(x0(1),x0(2),arrow_length*cos(x0(3)),arrow_length*sin(x0(3)),'off','Color',[0 0.4470 0.7410],'MaxHeadSize',0.5)
q = quiver(0,0,arrow_length*cos(0),arrow_length*sin(0),'off','Color',[0.6980 0.8314 0.9216],'MaxHeadSize',0.5);
clear p_RS
for t = 1:options.Hp
    [RS_local_x,RS_local_y] = boundary(mpa.local_reachable_sets{trim_root,t});
    p_RS(t) = plot(RS_local_x,RS_local_y,options_RS_local{t});
    [RS_global_x,RS_global_y] = translate_global(x0(3),x0(1),x0(2),RS_local_x',RS_local_y');
    p_RS(t) = plot(RS_global_x,RS_global_y,options_RS_global{t});
end
legend(p,{'Vehicle at trim 1 (local)','Vehicle at trim 1 (global)'},'Location','northeast')
EvaluationParl.save_fig(fig,file_name)