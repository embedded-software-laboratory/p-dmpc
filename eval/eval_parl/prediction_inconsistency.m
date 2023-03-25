%% evaluate the strategy of avoiding the problem of prediction inconsistency
isDealPredictionInconsistency = [true,false];
% prepare simulation options
options = Config();
options.is_sim_lab = true;
options.customResultName = '';
options.scenario_name = 'Commonroad';
options.trim_set = 9;
options.Hp = 5;

options.T_end = 4;
options.dt = 0.2;
options.max_num_CLs = 1;
options.priority = 'STAC_priority';
options.isPB = true;
options.isParl = true;
options.isAllowInheritROW = false;
options.isSaveResult = true;
options.isSaveResultReduced = false;
options.visu = [true,false];
options.is_eval = false;
options.strategy_consider_veh_without_ROW = '1';
options.strategy_enter_lanelet_crossing_area = '1';
manual_control_config = ManualControlConfig;
manual_control_config.amount = 0;
manual_control_config.hdv_ids = [];
options.manual_control_config = manual_control_config;

options.reference_path.lanelets_index = {[95,69,64,62,75,55,53],[76,24,13,15,3,5],[12,73,92,94,100,101]};
options.reference_path.start_point = [5,1,11];
options.amount = length(options.reference_path.lanelets_index);
options.veh_ids = 1:options.amount;
results = cell(2,1);
for i = 1:2
    options.isDealPredictionInconsistency = isDealPredictionInconsistency(i);
    full_path = FileNameConstructor.get_results_full_path(options,options.amount);
    if isfile(full_path)
        disp('File already exists.')
    else
        % run simulation
%         if exist('options','var') && exist('scenario','var')
%             [~,~] = main(options,scenario);
%         else
            [~,scenario] = main(options);
%         end
    end
    load(full_path,'result');
    results{i} = result;
end
disp('--------Finished--------')
%% test
options.isSaveResult = false;
options.visu = [true,false];
options.isDealPredictionInconsistency = false;
if exist('options','var') && exist('scenario','var')
    [~,~,~] = main(options,scenario);
else
    [~,scenario,~] = main(options);
end

%% Plot (consider reachable sets)
set(0,'DefaultTextFontname', 'Times New Roman');
set(0,'DefaultAxesFontName', 'Times New Roman');
set(0,'defaultTextFontSize',9)
set(0,'defaultAxesFontSize',9)

fig_x = 18;     fig_y = 5.8; % [cm]
x_margin = 0;   y_margin = 0; 
fig_x_position = fig_x - 2*x_margin;
fig_y_position = fig_y - 2*y_margin;

file_name = 'evalPaperConsiderReachableSets';
fig = figure('Name',file_name);
set(fig, 'Units','centimeters', 'Position',[0 0 fig_x_position fig_y_position]/2)
set(fig, 'PaperUnits','centimeters','PaperSize',[fig_x fig_y],'PaperOrientation','portrait',...
    'PaperPosition', [0.04 0.04 fig_x_position fig_y_position-0.08])

tiledlayout(1,3,'Padding','tight','TileSpacing','none');
vehColor = linspecer(8);
vehColor = vehColor(1:3,:);

tick_now = 1;
step_idx = 5;  
i = 1;
scenario_tmp = results{i}.scenarios{1};
nVehs = scenario_tmp.options.amount;

legend_hoziron = {'1st horizon','2nd horizon','3rd horizon','4th horizon','5th horizon'};
% plot footprints
nexttile(1,[1,1])
hold on
axis equal
xlim([1.6 4]);
ylim([1.05 2.75]);
daspect([1 1 1])
xticks('')
yticks('')
plot_lanelets(scenario_tmp.road_raw_data.lanelet,scenario_tmp.name);
% plot_boundary(results{i}.scenarios{1},vehColor)
% plot_boundary(results{i}.scenarios{9},vehColor,1)
for v = 1:nVehs
    for k = 1:2:13
        veh = scenario_tmp.vehicles(v);
        pos_step = results{i}.trajectory_predictions{v,k};
        x = pos_step(tick_now,:);
        vehiclePolygon = transformedRectangle(x(1),x(2),x(3), veh.Length,veh.Width);
        patch(   vehiclePolygon(1,:)...
                ,vehiclePolygon(2,:)...
                ,vehColor(v,:)...
                ,'LineWidth',0.2 ...
        );
    end
    for k = 1:2:13
        veh = scenario_tmp.vehicles(v);
        pos_step = results{i}.trajectory_predictions{v,k};
        x = pos_step(tick_now,:);
        vehiclePolygon = transformedRectangle(x(1),x(2),x(3), veh.Length,veh.Width);
        if k==1
            text(x(1),x(2),['{\it k}=' num2str(k)],'LineWidth',0.2,'Color','k','FontSize',9);
        else
            text(x(1),x(2),num2str(k),'LineWidth',0.2,'Color','k','FontSize',9);
        end

        % print vehicle ID
        if k==1
            text(x(1)+0.1,x(2)+0.1,num2str(v),'LineWidth',0.2,'Color',vehColor(v,:),'FontSize',11);
        end
    end
end
box on
xlabel('(a) Footprints.')

xmin = 2.1; xmax = 3.2;
ymin = 1.35; ymax = 2.35;

nexttile(2,[1,1])
hold on
box on
axis equal
xlim([xmin,xmax])
ylim([ymin,ymax])
daspect([1 1 1])
xticks('')
yticks('')

plot_lanelets(scenario_tmp.road_raw_data.lanelet,scenario_tmp.name);
% plot_boundary(scenario_tmp,vehColor)

for v=1:nVehs
    veh = scenario_tmp.vehicles(v);
    pos_step = results{i}.trajectory_predictions{v,step_idx};
    x = pos_step(tick_now,:);
    vehiclePolygon = transformedRectangle(x(1),x(2),x(3), veh.Length,veh.Width);
    patch(   vehiclePolygon(1,:)...
            ,vehiclePolygon(2,:)...
            ,vehColor(v,:)...
            ,'LineWidth',0.2 ...
    );
end

ego_vehs = 3;
other_vehs = setdiff(1:nVehs,ego_vehs);
% plot reachable set
for v=other_vehs
    reachable_sets = results{i}.iteration_structs{step_idx}.reachable_sets(v,:);
    reachable_sets_array = cellfun(@(c) {[c.Vertices(:,1)',c.Vertices(1,1)';c.Vertices(:,2)',c.Vertices(1,2)']}, reachable_sets); 
    plot_cell_arrays(reachable_sets_array)
end
shapes_ego = results{i}.shapes{step_idx}(ego_vehs,:);
p = plot_cell_arrays(shapes_ego);
legend(p,legend_hoziron,'FontSize',7,'Location','best')

for v=1:nVehs
    pos_step = results{i}.trajectory_predictions{v,step_idx};
    x = pos_step(tick_now,:);
    % plot the vehicle index
    text(x(1)+0.1,x(2)+0.1,num2str(v), 'LineWidth',0.2,'Color',vehColor(v,:),'FontSize',11);
end
xlabel('(b) Viewpoint of vehicle 3 at {\it k} = 5.')


nexttile(3,[1,1])
hold on
box on
axis equal
xlim([xmin,xmax])
ylim([ymin,ymax])
daspect([1 1 1])
xticks('')
yticks('')

plot_lanelets(scenario_tmp.road_raw_data.lanelet,scenario_tmp.name);
% plot_boundary(scenario_tmp,vehColor)
for v=1:nVehs
    veh = scenario_tmp.vehicles(v);
    pos_step = results{i}.trajectory_predictions{v,step_idx};
    x = pos_step(tick_now,:);
    vehiclePolygon = transformedRectangle(x(1),x(2),x(3), veh.Length,veh.Width);
    patch(   vehiclePolygon(1,:)...
            ,vehiclePolygon(2,:)...
            ,vehColor(v,:)...
            ,'LineWidth',0.2 ...
    );
end

ego_vehs = 1:nVehs;
other_vehs = setdiff(1:nVehs,ego_vehs);
% plot one-step-shifted previous plans of other vehicles
for v=other_vehs
    shapes = del_first_rpt_last(results{i}.shapes{step_idx}(v,:),1);
    plot_cell_arrays(shapes)
end
% plot current plans of the ego vehicles
for v=ego_vehs
    shapes_ego = results{i}.shapes{step_idx}(v,:);
    p = plot_cell_arrays(shapes_ego);
end
legend(p,legend_hoziron,'FontSize',7,'Location','best')

for v=1:nVehs
    pos_step = results{i}.trajectory_predictions{v,step_idx};
    x = pos_step(tick_now,:);
    % plot the vehicle index
    text(x(1)+0.1,x(2)+0.1,num2str(v), 'LineWidth',0.2,'Color',vehColor(v,:),'FontSize',11);
end
xlabel('(c) Actual planned trajectories at {\it k} = 5.')

% save fig
EvaluationParl.save_fig(fig,file_name)

%% Plot (consider previous trajectories)
set(0,'DefaultTextFontname', 'Times New Roman');
set(0,'DefaultAxesFontName', 'Times New Roman');
set(0,'defaultTextFontSize',9)
set(0,'defaultAxesFontSize',9)

fig_x = 18;     fig_y = 5.8; % [cm]
x_margin = 0;   y_margin = 0; 
fig_x_position = fig_x - 2*x_margin;
fig_y_position = fig_y - 2*y_margin;

file_name = 'evalPaperConsiderPreviousPlans';
% file_name = 'evalPaperConsiderReachableSets';
fig = figure('Name',file_name);
set(fig, 'Units','centimeters', 'Position',[0 0 fig_x_position fig_y_position]/2)
set(fig, 'PaperUnits','centimeters','PaperSize',[fig_x fig_y],'PaperOrientation','portrait',...
    'PaperPosition', [0.04 0.04 fig_x_position fig_y_position-0.08])

tiledlayout(1,3,'Padding','tight','TileSpacing','none');
vehColor = linspecer(8);
vehColor = vehColor(1:3,:);
nVehs = scenario_tmp.options.amount;
tick_now = 1;
i = 2;
scenario_tmp = results{i}.scenario;

% print footprints
nexttile
hold on
axis equal
xlim([1.6 4]);
ylim([1.05 2.75]);
daspect([1 1 1])
xticks('')
yticks('')
plot_lanelets(scenario_tmp.road_raw_data.lanelet,scenario_tmp.name);
% plot_boundary(results{i}.scenarios{1},vehColor)
% plot_boundary(results{i}.scenarios{end},vehColor,1)
for v = 1:nVehs
    for k = 1:2:size(results{i}.trajectory_predictions,2)
        veh = scenario_tmp.vehicles(v);
        pos_step = results{i}.trajectory_predictions{v,k};
        x = pos_step(tick_now,:);
        vehiclePolygon = transformedRectangle(x(1),x(2),x(3), veh.Length,veh.Width);
        patch(   vehiclePolygon(1,:)...
                ,vehiclePolygon(2,:)...
                ,vehColor(v,:)...
                ,'LineWidth',0.2 ...
        );
    end
    for k = 1:2:size(results{i}.trajectory_predictions,2)
        veh = scenario_tmp.vehicles(v);
        pos_step = results{i}.trajectory_predictions{v,k};
        x = pos_step(tick_now,:);
        vehiclePolygon = transformedRectangle(x(1),x(2),x(3), veh.Length,veh.Width);
        if k==1
            text(x(1),x(2),['{\it k}=' num2str(k)],'LineWidth',0.2,'Color','k','FontSize',9);
        else
            text(x(1),x(2),num2str(k),'LineWidth',0.2,'Color','k','FontSize',9);
        end

        % print vehicle ID
        if k==1
            text(x(1)+0.1,x(2)+0.1,num2str(v),'LineWidth',0.2,'Color',vehColor(v,:),'FontSize',11);
        end
    end
    if i==2
        x = pos_step(tick_now+(scenario_tmp.options.tick_per_step+1)*2,:);
        vehiclePolygon = transformedRectangle(x(1),x(2),x(3), veh.Length,veh.Width);
        patch(   vehiclePolygon(1,:)...
                ,vehiclePolygon(2,:)...
                ,vehColor(v,:)...
                ,'LineWidth',0.2 ...
        );
        text(x(1),x(2),num2str(k+2),'LineWidth',0.2,'Color','k','FontSize',9);
    end
end
xlabel('(a) Footprints.')
box on

step_idx = 5;    
tick_now = 1;
xmin = 2.1; xmax = 3.2;
ymin = 1.35; ymax = 2.35;

legend_hoziron = {'1st horizon','2nd horizon','3rd horizon','4th horizon','5th horizon'};
nexttile(2)
hold on
axis equal
xlim([xmin,xmax])
ylim([ymin,ymax])
daspect([1 1 1])
xticks('')
yticks('')

plot_lanelets(scenario_tmp.road_raw_data.lanelet,scenario_tmp.name);
% plot_boundary(scenario_tmp,vehColor)

% plot current positions
for v=1:nVehs
    veh = scenario_tmp.vehicles(v);
    pos_step = results{i}.trajectory_predictions{v,step_idx};
    x = pos_step(tick_now,:);
    vehiclePolygon = transformedRectangle(x(1),x(2),x(3), veh.Length,veh.Width);
    patch(   vehiclePolygon(1,:)...
            ,vehiclePolygon(2,:)...
            ,vehColor(v,:)...
            ,'LineWidth',0.2 ...
    );
end

ego_vehs = 3;
other_vehs = setdiff(1:nVehs,ego_vehs);
% plot one-step-shifted previous plans
for v=other_vehs
    shapes = del_first_rpt_last(results{i}.shapes{step_idx-1}(v,:),1);
    plot_cell_arrays(shapes);
%     plot_cell_arrays(shapes,vehColor(v,:),true)
end
shapes_ego = results{i}.shapes{step_idx}(ego_vehs,:);
p = plot_cell_arrays(shapes_ego);

legend(p,{'1st horizon','2nd horizon','3rd horizon','4th horizon','5th horizon'},'FontSize',7,'Location','best')

for v=1:nVehs
    pos_step = results{i}.trajectory_predictions{v,step_idx};
    x = pos_step(tick_now,:);
    % plot the vehicle index
    text(x(1)+0.1,x(2)+0.1,num2str(v), 'LineWidth',0.2,'Color',vehColor(v,:),'FontSize',11);
end
box on
xlabel('(b) Viewpoint of vehicle 3 at {\it k} = 5.')

nexttile(3)
hold on
axis equal
xlim([xmin,xmax])
ylim([ymin,ymax])
daspect([1 1 1])
xticks('')
yticks('')

plot_lanelets(scenario_tmp.road_raw_data.lanelet,scenario_tmp.name);
% plot_boundary(scenario_tmp,vehColor)

% plot current positions
for v=1:nVehs
    veh = scenario_tmp.vehicles(v);
    pos_step = results{i}.trajectory_predictions{v,step_idx};
    x = pos_step(tick_now,:);
    vehiclePolygon = transformedRectangle(x(1),x(2),x(3), veh.Length,veh.Width);
    patch(   vehiclePolygon(1,:)...
            ,vehiclePolygon(2,:)...
            ,vehColor(v,:)...
            ,'LineWidth',0.2 ...
    );
end

ego_vehs = 1:nVehs;
other_vehs = setdiff(1:nVehs,ego_vehs);
% plot one-step-shifted previous plans of other vehicles
for v=other_vehs
    shapes = del_first_rpt_last(results{i}.shapes{step_idx-1}(v,:),1);
    plot_cell_arrays(shapes);
end
% plot current plans of the ego vehicles
for v=ego_vehs
    shapes_ego = results{i}.shapes{step_idx}(v,:);
    p = plot_cell_arrays(shapes_ego); 
end
legend(p,legend_hoziron,'FontSize',7,'Location','best')

for v=1:nVehs
    pos_step = results{i}.trajectory_predictions{v,step_idx};
    x = pos_step(tick_now,:);
    % plot the vehicle index
    text(x(1)+0.1,x(2)+0.1,num2str(v), 'LineWidth',0.2,'Color',vehColor(v,:),'FontSize',11);
end
xlabel('(c) Actual planned trajectories at {\it k} = 5.')
box on

% save fig
EvaluationParl.save_fig(fig,file_name)
%% helper function
function plot_boundary(scenario_tmp,vehColor,veh_ego)
    if nargin==2
        veh_ego = [];
    end
    if isempty(veh_ego)
        for v = 1:scenario_tmp.options.amount
            lanelet_boundary = scenario_tmp.vehicles(v).lanelet_boundary;
            line(lanelet_boundary{1}(1,:),lanelet_boundary{1}(2,:),'Color',vehColor(v,:),'LineStyle','-','LineWidth',.8)
            line(lanelet_boundary{2}(1,:),lanelet_boundary{2}(2,:),'Color',vehColor(v,:),'LineStyle','-','LineWidth',.8) 
        end
    else
        v = veh_ego;
        lanelet_boundary = scenario_tmp.vehicles(v).lanelet_boundary;
        line(lanelet_boundary{1}(1,:),lanelet_boundary{1}(2,:),'Color',vehColor(v,:),'LineStyle','-','LineWidth',.8)
        line(lanelet_boundary{2}(1,:),lanelet_boundary{2}(2,:),'Color',vehColor(v,:),'LineStyle','-','LineWidth',.8) 
    end
end
%% helper function
function p = plot_cell_arrays(cells,color,isFill)
% Plot shapes contained in a cell array
%     CM = jet(Hp); % colormap
    hold on
    if nargin==1
        isFill = false;
    end
    if isFill
        for j = 1:size(cells,2)
            shape = cells{j};
            patch(shape(1,:),shape(2,:),color,'FaceAlpha',j/(size(cells,2)+2),'EdgeColor','none');
        end
    else
        CM = linspecer(size(cells,2)+3);
        CM = CM(4:end,:);
    
        for j = 1:size(cells,2)
            shape = cells{j};
            p(j) = plot(shape(1,:),shape(2,:),'LineWidth',1,'Color',CM(j,:),'LineStyle','-.');
        end
    end
end