%% Generate data: different allowed number of computation levels and different sample time
% random simulations
options = Config();
options.is_sim_lab = true;
options.customResultName = '';
options.scenario_name = 'Commonroad';
options.trim_set = 9;
options.priority = 'STAC_priority';
options.isPB = true;
options.isParl = true;
options.isAllowInheritROW = false;
options.strategy_consider_veh_without_ROW = '3';
options.strategy_enter_lanelet_crossing_area = '1';
options.dt = 0.2;

options.isSaveResult = true;
options.visu = [false,false];
options.Hp = 7;
options.T_end = 4;
options.amount = 20;

random_times = 10;
CLs_s = [1,2,4,6,options.amount];
e_CLs = cell(random_times,length(CLs_s));
results = cell(random_times,length(CLs_s));
n_simulations = numel(e_CLs);
count = 0;

random_seed = RandStream('mt19937ar');
for i = 1:random_times
    options.random_idx = i;
    for j = 1:length(CLs_s)
        if i==1 && j==1
            options.isSaveResultReduced = false;
        else
            options.isSaveResultReduced = true;
        end
        options.max_num_CLs = CLs_s(j);
        options.veh_ids = sort(randsample(random_seed,1:40,options.amount),'ascend');
        full_path = FileNameConstructor.get_results_full_path(options, options.amount);
        
        if isfile(full_path)
            disp('File already exists.')
        else
            % run simulation
            if exist('options','var') && exist('scenario','var')
                [~,~] = main(options,scenario);
            else
                [~,scenario] = main(options);
            end
            pause(3)
        end
        % data processing
        e_CLs{i,j} = EvaluationParl(options);
        load(full_path,'result')
        results{i,j} = result;
        
        % display progress
        count = count + 1;
        disp(['--------Progress ' num2str(count) '/' num2str(n_simulations) ': done--------'])
    end
end

% get free flow speed, i.e., the speed that vehicles could travel if they are not influenced by others
% vehicles
if exist('Scenario','var')
    free_flow_speed = FreeFlowSpeed(scenario);
else
    free_flow_speed = FreeFlowSpeed();
end
free_flow_speed_i = free_flow_speed.free_flow_speed(free_flow_speed.sample_time==options.dt);
disp('--------Finished--------')
%% test
options = OptionsMain;
options.is_sim_lab = true;
options.customResultName = '';
options.scenario_name = 'Commonroad';
options.trim_set = 9;
options.priority = 'STAC_priority';
options.isPB = true;
options.isParl = true;
options.isAllowInheritROW = false;
options.strategy_consider_veh_without_ROW = '3';
options.strategy_enter_lanelet_crossing_area = '1';
options.dt = 0.2;

options.isSaveResult = false;
options.visu = [true,false];
options.Hp = 7;
options.T_end = 4;
options.amount = 20;
options.max_num_CLs = 1;

random_seed = RandStream('mt19937ar');
options.veh_ids = sort(randsample(random_seed,1:40,options.amount),'ascend');
% options.amount = numel(options.veh_ids);
% options.veh_ids = [11,18,19,20,26,27,29,31,32,40];
% options.veh_ids = [11,18,20,27,32,40];
% options.amount = numel(options.veh_ids);

count = 1;

% run simulation
if exist('options','var') && exist('scenario','var')
    [~,~,~] = main(options,scenario);
else
    [~,scenario,~] = main(options);
end
%% plot random all
set(0,'DefaultTextFontname', 'Times New Roman');
set(0,'DefaultAxesFontName', 'Times New Roman');
set(0,'defaultTextFontSize',11)
set(0,'defaultAxesFontSize',11)

average_random_simulations = @(data) reshape(mean(reshape(data,random_times,[]),1),length(strategy_feasibility_deadlock),[]);

CT_graph_search = cellfun(@(c) c.runtime_graph_search, e_CLs);  
CT_graph_search_median = cellfun(@(c) c.runtime_graph_search_median, e_CLs);
% CT_graph_search_pure_sequential = cellfun(@(c) c.runtime_graph_search_pure_sequential, e_CLs);  
% CT_graph_search_pure_sequential_average = cellfun(@(c) c.runtime_graph_search_pure_sequential_average, e_CLs);
% CT_graph_search(end+1) = CT_graph_search_pure_sequential(end);
% CT_graph_search_average(end+1) = CT_graph_search_pure_sequential_average(end);
CT_total = cellfun(@(c) c.runtime_total, e_CLs); 
CT_total_median = cellfun(@(c) c.runtime_total_median, e_CLs);
% CT_total_pure_squential = cellfun(@(c) c.runtime_total_pure_sequential, e_CLs);
% CT_total_pure_squential_average = cellfun(@(c) c.runtime_total_pure_sequential_average, e_CLs);
% CT_total(end+1) = CT_total_pure_squential(end);
% CT_total_average(end+1) = CT_total_pure_squential_average(end);

% maximum number of actual computation levels
CLs_num_max = cellfun(@(c) c.CLs_num_max, e_CLs);

nodes_expended = cellfun(@(c) c.nodes_expanded, e_CLs);
nodes_expended_max = cellfun(@(c) c.nodes_expanded_max, e_CLs);
nodes_expended_median = cellfun(@(c) c.nodes_expanded_median, e_CLs);

plot_line_options = {};
plot_line_options{1}{1} = struct('LineWidth',0.6,'Color','#A2142F','LineStyle','-','Marker','*','MarkerSize',4); % med.
plot_line_options{1}{2} = struct('LineWidth',0.6,'Color','#A2142F','LineStyle','-.','Marker','*','MarkerSize',4);% max.
plot_line_options{2}{1} = struct('LineWidth',0.6,'Color','#7E2F8E','LineStyle','-','Marker','^','MarkerSize',4);
plot_line_options{2}{2} = struct('LineWidth',0.6,'Color','#7E2F8E','LineStyle','-.','Marker','^','MarkerSize',4);
plot_line_options{3}{1} = struct('LineWidth',0.6,'Color','#0072BD','LineStyle','-','Marker','o','MarkerSize',4);
plot_line_options{3}{2} = struct('LineWidth',0.6,'Color','#0072BD','LineStyle','-.','Marker','o','MarkerSize',4);

fig_x = 50;     fig_y = 30; % [cm]
x_margin = 0;   y_margin = 0; 
fig_x_position = fig_x - 2*x_margin;
fig_y_position = fig_y - 2*y_margin;

file_name = ['evalPaperCLsRandomsHp' num2str(options.Hp)];
fig = figure('Name',file_name);
set(fig, 'Units','centimeters', 'Position',[0 0 fig_x_position fig_y_position]/2)
set(fig, 'PaperUnits','centimeters','PaperSize',[fig_x fig_y],'PaperOrientation','portrait',...
    'PaperPosition', [0 0 fig_x_position fig_y_position])

t_fig = tiledlayout(3,10,'Padding','tight','TileSpacing','tight');

X_string = string(CLs_s);
X_string(1) = 'Parl.';
X_string(2) = ['$N_{l,a}=$' num2str(CLs_s(2))];
X_string(end) = 'Alr.';
X_cat = categorical(X_string);
X_cat = reordercats(X_cat,X_string);

for i = 1:random_times
    % plot computation time
    % maximum allowed and actual number of computation levels
    nexttile(i)
    grid on 
    hold on
    clear p_CLs
    p_CLs(1) = plot(X_cat,CLs_s,plot_line_options{1}{1});
    p_CLs(2) = plot(X_cat,CLs_num_max(i,:),plot_line_options{2}{1});
    ylim([0 20])
    yticks(0:5:20)
    legend(p_CLs,{'Allowed','Actual'},'FontSize',9,'Location','north')
%     xlabel({'(c)'},'Interpreter','latex');
    ylabel('$N_{l}$','Interpreter','latex')
    xaxis = get(gca, 'XAxis');
    xaxis.TickLabelInterpreter = 'latex'; % latex for x-axis

    nexttile(i+random_times)
    grid on
    hold on
    clear p
    p(1) = plot(X_cat,CT_total(i,:),plot_line_options{1}{2});
    p(2) = plot(X_cat,CT_total_median(i,:),plot_line_options{1}{1});
    p(3) = plot(X_cat,CT_graph_search(i,:),plot_line_options{3}{2});
    p(4) = plot(X_cat,CT_graph_search_median(i,:),plot_line_options{3}{1});
    p(5) = yline(options.dt,'--b','LineWidth',0.6);
    ylabel('$t_c\:[s]$','Interpreter','latex')
    tmax = 1.2;
    ylim([0,tmax])
    yticks(0:0.1:tmax)
    xaxis = get(gca, 'XAxis');
    xaxis.TickLabelInterpreter = 'latex'; % latex for x-axis
    legend(p,{'Total (max.)','Total (med.)','Trajectory (max)','Trajectory (med.)','Sample time'},'Location','north');

    nexttile(i+random_times*2)
    grid on
    hold on
    clear p
    p(1) = plot(X_cat,nodes_expended_max(i,:),plot_line_options{2}{2});
    p(2) = plot(X_cat,nodes_expended(i,:),plot_line_options{2}{2});
    p(3) = plot(X_cat,nodes_expended_median(i,:),plot_line_options{2}{1});
    legend(p,{'Max.','Med.'},'Location','north');
%     bp = gca;
%     bp.XAxis.TickLabelInterpreter = 'latex';
    xaxis = get(gca, 'XAxis');
    xaxis.TickLabelInterpreter = 'latex'; % latex for x-axis
    grid on
    ylabel('# of expended nodes')
    ylim([0 4.5e4])
    xlabel(['Random ' num2str(i)],'Interpreter','latex');
end
title(t_fig,['Hp: ' num2str(options.Hp) ', nVeh: ' num2str(options.amount), ', T: ' num2str(options.T_end) ' secconds'])

% xlabel({'Random index'},'Interpreter','latex');
% ylabel('$t_c\:[s]$','Interpreter','latex')

% save fig
e_CLs{1}.save_fig(fig,file_name)
%% plot random each
set(0,'DefaultTextFontname', 'Times New Roman');
set(0,'DefaultAxesFontName', 'Times New Roman');
set(0,'defaultTextFontSize',10)
set(0,'defaultAxesFontSize',10)

average_random_simulations = @(data) reshape(mean(reshape(data,random_times,[]),1),length(CLs_s),[]);

CT_graph_search = cellfun(@(c) c.runtime_graph_search, e_CLs);

CT_graph_search_median = cellfun(@(c) c.runtime_graph_search_median, e_CLs);
% CT_graph_search_pure_sequential = cellfun(@(c) c.runtime_graph_search_pure_sequential, e_CLs);  
% CT_graph_search_pure_sequential_average = cellfun(@(c) c.runtime_graph_search_pure_sequential_average, e_CLs);
% CT_graph_search(end+1) = CT_graph_search_pure_sequential(end);
% CT_graph_search_average(end+1) = CT_graph_search_pure_sequential_average(end);
CT_total = cellfun(@(c) c.runtime_total, e_CLs); 
CT_total_median = cellfun(@(c) c.runtime_total_median, e_CLs);
% CT_total_pure_squential = cellfun(@(c) c.runtime_total_pure_sequential, e_CLs);
% CT_total_pure_squential_average = cellfun(@(c) c.runtime_total_pure_sequential_average, e_CLs);
% CT_total(end+1) = CT_total_pure_squential(end);
% CT_total_average(end+1) = CT_total_pure_squential_average(end);

% maximum number of actual computation levels
CLs_num_max = cellfun(@(c) c.CLs_num_max, e_CLs);

% average speed

average_speed_s = cellfun(@(c) c.average_speed, e_CLs);
average_speed_normalized = average_speed_s./free_flow_speed_i;
average2_speed_normalized = mean(average_speed_normalized);

nodes_expended = cellfun(@(c) c.nodes_expanded, e_CLs);
nodes_expended_max = cellfun(@(c) c.nodes_expanded_max, e_CLs);
nodes_expended_median = cellfun(@(c) c.nodes_expanded_median, e_CLs);

plot_line_options = {};
plot_line_options{1}{1} = struct('LineWidth',0.6,'Color','#A2142F','LineStyle','-','Marker','*','MarkerSize',4); % med.
plot_line_options{1}{2} = struct('LineWidth',0.6,'Color','#A2142F','LineStyle','-.','Marker','*','MarkerSize',4);% max.
plot_line_options{2}{1} = struct('LineWidth',0.6,'Color','#7E2F8E','LineStyle','-','Marker','^','MarkerSize',4);
plot_line_options{2}{2} = struct('LineWidth',0.6,'Color','#7E2F8E','LineStyle','-.','Marker','^','MarkerSize',4);
plot_line_options{3}{1} = struct('LineWidth',0.6,'Color','#0072BD','LineStyle','-','Marker','o','MarkerSize',4);
plot_line_options{3}{2} = struct('LineWidth',0.6,'Color','#0072BD','LineStyle','-.','Marker','o','MarkerSize',4);

fig_x = 7;     fig_y = 8; % [cm]
x_margin = 0;   y_margin = 0; 
fig_x_position = fig_x - 2*x_margin;
fig_y_position = fig_y - 2*y_margin;

file_name = 'evalPaperCLs';
fig = figure('Name',file_name);
set(fig, 'Units','centimeters', 'Position',[0 0 fig_x_position fig_y_position]/2)
set(fig, 'PaperUnits','centimeters','PaperSize',[fig_x fig_y],'PaperOrientation','portrait',...
    'PaperPosition', [0 0.02 fig_x_position-0.06 fig_y_position-0.04])

t_fig = tiledlayout(2,1,'Padding','tight','TileSpacing','tight');

X_string = string(CLs_s);
X_string(1) = 'Parl.';
X_string(2) = ['$N_{l,a}=$' num2str(CLs_s(2))];
X_string(end) = 'Alr.';
X_cat = categorical(X_string);
X_cat = reordercats(X_cat,X_string);

% plot average speed
nexttile(1)
grid on 
hold on
b1 = boxplot(average_speed_normalized*100,'Colors','k','OutlierSize',4,'Labels',X_string);
hAx = gca;                                
xtk = hAx.XTick;  

% sc = scatter(xtk,average2_speed_normalized,18,'filled','o','MarkerFaceColor',[0 0.4470 0.7410]);

y_FFS = yline(free_flow_speed_i/free_flow_speed_i*100,'--b','LineWidth',0.6);

legend(y_FFS,{'Free-flow'},'Location','south')
ylabel('$\overline{v}_n\:[\%]$','Interpreter','latex')
xlabel('(a) Average speed (normalized).')
xaxis = get(gca, 'XAxis');
xaxis.TickLabelInterpreter = 'latex'; % latex for x-axis

ylim([0 105])
yticks(0:20:105)


% plot computation levels
nexttile(2)
grid on 
hold on
clear p_CLs p_CLs_max
p_CLs_max = boxplot(CLs_num_max,'Colors','k','OutlierSize',4,'Labels',X_string);

% p_CLs = plot(X_cat,CLs_s,plot_line_options{1}{1});

level_max = 12;
ylim([0 level_max])
yticks(0:2:level_max)
% legend([p_CLs,p_CLs_max(end)],{'Allowed','Actual'},'FontSize',9,'Location','north')
%     xlabel({'(c)'},'Interpreter','latex');
xlabel('(b) Actual number of computation levels.')
ylabel('$N_{l}$','Interpreter','latex')
xaxis = get(gca, 'XAxis');
xaxis.TickLabelInterpreter = 'latex'; % latex for x-axis

% plot computation time
% nexttile(3)
% grid on
% hold on
% clear p
% 
% boxplot(CT_total./median(CT_total(:,end))*100,'Colors','k','OutlierSize',4,'Labels',X_string);
% % boxplot(CT_graph_search,'Colors','k','OutlierSize',4,'Labels',X_string);
% % yline(options.dt,'--b','LineWidth',0.6);
% xlabel('(c)')
% ylabel('$t_n\:[\%]$','Interpreter','latex')
% tmax = 260;
% ylim([0,tmax])
% yticks(0:50:tmax)
% xaxis = get(gca, 'XAxis');
% xaxis.TickLabelInterpreter = 'latex'; % latex for x-axis
% legend(p,{'Total','Trajectory (max)','Sample time'},'Location','north');


% xlabel({'Random index'},'Interpreter','latex');
% ylabel('$t_n\:[s]$','Interpreter','latex')

% save fig
e_CLs{1}.save_fig(fig,file_name)
%% graph search snapshot
if ~exist('result','var') && result.scenario.options.max_num_CLs==1 && result.scenario.options.random_idx==1
    options = OptionsMain;
    options.is_sim_lab = true;
    options.customResultName = '';
    options.scenario_name = 'Commonroad';
    options.trim_set = 9;
    options.priority = 'STAC_priority';
    options.isPB = true;
    options.isParl = true;
    options.isAllowInheritROW = false;
    options.strategy_consider_veh_without_ROW = '3';
    options.strategy_enter_lanelet_crossing_area = '1';
    options.dt = 0.2;
    
    options.isSaveResult = false;
    options.isSaveResultReduced = false;
    options.visu = [false,false];
    options.Hp = 7;
    options.T_end = 4;
    options.amount = 20;
    options.max_num_CLs = 1;
    
    full_path = FileNameConstructor.get_results_full_path(options);
    load(full_path)
end

set(0,'DefaultTextFontname', 'Times New Roman');
set(0,'DefaultAxesFontName', 'Times New Roman');
set(0,'defaultTextFontSize',11)
set(0,'defaultAxesFontSize',11)

fig_x = 4;     fig_y = 4; % [cm]
x_margin = 0;   y_margin = 0; 
fig_x_position = fig_x - 2*x_margin;
fig_y_position = fig_y - 2*y_margin;

file_name = 'evalPaperWorstCaseTimestep';
fig = figure('Name',file_name);
set(fig, 'Units','centimeters', 'Position',[0 0 fig_x_position fig_y_position]/2)
set(fig, 'PaperUnits','centimeters','PaperSize',[fig_x fig_y],'PaperOrientation','portrait',...
    'PaperPosition', [0 -0.45 fig_x_position fig_y_position+0.75])

tiledlayout(1,1,'Padding','tight','TileSpacing','none');
vehColor = linspecer(9);
vehColor = vehColor(1:2,:);

scenario_tmp = result.scenario;
tick_now = 1;
step_idx = 9;

hold on
axis equal

xlim([0 1]);
ylim([0.2 1.6]);
daspect([1 1 1])
xticks('')
yticks('')
plot_lanelets(scenario_tmp.road_raw_data.lanelet,scenario_tmp.name)
vehs = [11,2];

for i = 1:length(vehs)
    v = vehs(i);
    veh = scenario_tmp.vehicles(v);
    pos_step = result.trajectory_predictions{v,step_idx};
    x = pos_step(tick_now,:);
    vehiclePolygon = transformedRectangle(x(1),x(2),x(3), veh.Length,veh.Width);
    patch(   vehiclePolygon(1,:)...
            ,vehiclePolygon(2,:)...
            ,vehColor(i,:)...
            ,'LineWidth',0.2 ...
    );
end

ego_vehs = 2;
other_vehs = setdiff(vehs,ego_vehs);
% plot reachable set
for v = other_vehs
    reachable_sets = result.iteration_structs{step_idx}.reachable_sets(v,:);
    reachable_sets_array = cellfun(@(c) {[c.Vertices(:,1)',c.Vertices(1,1)';c.Vertices(:,2)',c.Vertices(1,2)']}, reachable_sets); 
    plot_cell_arrays(reachable_sets_array)
end
shapes_ego = result.shapes{step_idx}(ego_vehs,:);
p = plot_cell_arrays(shapes_ego);

for i = 1:length(vehs)
    v = vehs(i);
    pos_step = result.trajectory_predictions{v,step_idx};
    x = pos_step(tick_now,:);
    % plot the vehicle index
    text(x(1)+0.1,x(2)+0.1,num2str(i), 'LineWidth',0.2,'Color','b');
end

% Sampled trajectory points

line(   result.iteration_structs{step_idx}.referenceTrajectoryPoints(ego_vehs,:,1), ...
        result.iteration_structs{step_idx}.referenceTrajectoryPoints(ego_vehs,:,2), ...
        'Color',vehColor(ego_vehs,:),'LineStyle','none','Marker','o','MarkerFaceColor',vehColor(ego_vehs,:),'MarkerSize',2,'LineWidth',1 );

e_CLs{1}.save_fig(fig,file_name)
%% video
for i = 1:numel(results)
    result = results{i};

    result.scenario.options.optionsPlotOnline.isVideoMode = true;
    result.scenario.options.optionsPlotOnline.isShowCoupling = true;
    result.scenario.options.optionsPlotOnline.isShowPriority = true;
    
    % videoExportSetup.framerate = 30;
    exportVideo(result)
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