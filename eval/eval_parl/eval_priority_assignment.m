%% Evaluate the vehicle prioritizing algorithm: maximum number of vehicles
priority_assign_options = {'STAC_priority','random_priority','constant_priority'};

% prepare simulation options
options = OptionsMain;
options.consider_RSS = false;
options.is_sim_lab = true;
options.customResultName = '';
options.scenario_name = 'Commonroad';
options.trim_set = 9;
options.Hp = 5;
options.dt = 0.2;
options.T_end = 20;
options.isPB = true;
options.isAllowInheritROW = false;
options.max_num_CLs = 4;
options.strategy_consider_veh_without_ROW = '3';
options.strategy_enter_lanelet_crossing_area = '4';
options.isSaveResult = true;
options.visu = [false,false];
options.is_eval = false;
options.visualize_reachable_set = false;

e_differentNumVehs = cell(length(priority_assign_options),1);
n_simulations = numel(e_differentNumVehs);
count = 0;

for i_priority = 1:length(priority_assign_options)
    options.priority = priority_assign_options{i_priority};
    % After preexamining, maximum 15 (18) vehicles can run collision- and deadlock-free
    % if random (constant) priority assignment is used
    switch options.priority
        case 'STAC_priority'
            options.amount = 40;
        case 'random_priority'
            options.amount = 16;
        case 'constant_priority'
            options.amount = 16;
    end

    random_stream = RandStream('mt19937ar');
    options.veh_ids = sort(randsample(random_stream,1:40,options.amount),'ascend');

    results_full_path = FileNameConstructor.get_results_full_path(options);
    if isfile(results_full_path)
        disp('File already exists.')
    else
        % run simulation
        if exist('options','var') && exist('scenario','var')
            [~,~,~] = main(options,scenario);
        else
            [~,scenario,~] = main(options);
        end
        pause(10) % pause to cool the machine
    end

    % evaluate
    e_differentNumVehs{i_priority} = EvaluationParl(results_full_path,[0,options.T_end]);
    
    % display progress
    count = count + 1;
    disp(['--------Progress ' num2str(count) '/' num2str(n_simulations) ': done--------'])
end
disp('--------Finished--------')


%% Plot
set(0,'DefaultTextFontname', 'Times New Roman');
set(0,'DefaultAxesFontName', 'Times New Roman');
set(0,'defaultTextFontSize',11)
set(0,'defaultAxesFontSize',11)

nVeh_s = cellfun(@(c) c.nVeh, e_differentNumVehs);
speed_average_s = cellfun(@(c) c.average_speed, e_differentNumVehs);
speed_sum_s = cellfun(@(c) c.sum_speeds_all_vehicles, e_differentNumVehs);

fig_x = 15;     fig_y = 6; % [cm]
x_margin = 0.1;   y_margin = 0.05; 
fig_x_position = fig_x - x_margin;
fig_y_position = fig_y;

file_name = 'evalPriorityAssignment_differentNumVehs';
fig = figure('Name',file_name);
set(fig, 'Units','centimeters', 'Position',[0 0 fig_x_position fig_y_position]/2)
set(fig, 'PaperUnits','centimeters','PaperSize',[fig_x fig_y],'PaperOrientation','portrait',...
    'PaperPosition', [0 y_margin fig_x_position fig_y_position])

t_fig = tiledlayout(1,2,'Padding','tight','TileSpacing','compact');
X_string = {'STAC-based','Random','Constant'};
X_cat = categorical(X_string);
X_cat = reordercats(X_cat,X_string);

% maximum controllable vehicles
nexttile
b1 = bar(X_cat,nVeh_s);
grid on
xtips1 = b1(1).XEndPoints;
ytips1 = b1(1).YEndPoints;
labels1 = string(b1(1).YData);
text(xtips1,ytips1,labels1,'HorizontalAlignment','center','VerticalAlignment','bottom')
xlabel({'(a) Maximum controllable vehicles.'})
ylabel('$n_{veh}^{max}$','Interpreter','latex')
ylim([0 48])
xtickangle(0)

% average speed
nexttile
b2 = bar(X_cat,[speed_average_s,speed_sum_s]);
grid on
xtips1 = b2(1).XEndPoints;
ytips1 = b2(1).YEndPoints;
labels1 = string(round(b2(1).YData,2));
text(xtips1-0.02,ytips1,labels1,'HorizontalAlignment','center','VerticalAlignment','bottom')
xtips2 = b2(2).XEndPoints;
ytips2 = b2(2).YEndPoints;
labels2 = string(round(b2(2).YData,2));
text(xtips2,ytips2,labels2,'HorizontalAlignment','center','VerticalAlignment','bottom')
xlabel('(b) Average speed and sum of speeds.')
ylabel('$\overline{v},\overline{v}_{sum}\:[m/s]$','Interpreter','latex')
legend({'Average','Sum'},'Location','northeast')
ylim([0 25])
xtickangle(0)

% save fig
e_differentNumVehs{1}.save_fig(fig,file_name)
%% Evaluate the vehicle prioritizing algorithm: use the same number of vehicle
priority_assign_options = {'STAC_priority','random_priority','constant_priority'};
% prepare simulation options
options = OptionsMain;
options.consider_RSS = false;
options.is_sim_lab = true;
options.customResultName = '';
options.scenario_name = 'Commonroad';
options.trim_set = 9;
options.Hp = 5;
options.dt = 0.2;
options.T_end = 20;
options.isPB = true;
options.isAllowInheritROW = false;
options.max_num_CLs = 4;
options.strategy_consider_veh_without_ROW = '3';
options.strategy_enter_lanelet_crossing_area = '4';
options.isSaveResult = true;
options.visu = [false,false];
options.is_eval = false;
options.visualize_reachable_set = false;
options.amount = 16;

random_stream = RandStream('mt19937ar');
options.veh_ids = sort(randsample(random_stream,1:40,options.amount),'ascend');

e_sameNumVehs = cell(length(priority_assign_options),1);
n_simulations = numel(e_sameNumVehs);
count = 0;

for i_priority = 1:length(priority_assign_options)
    options.priority = priority_assign_options{i_priority};
    % Use the maximum number of vehicles that are controllable by all the three
    % priority assignment strategies
    results_full_path = FileNameConstructor.get_results_full_path(options);
    if isfile(results_full_path)
        disp('File already exists.')
    else
        % run simulation
        if exist('options','var') && exist('scenario','var')
            [~,~,~] = main(options,scenario);
        else
            [~,scenario,~] = main(options);
        end
        pause(10) % pause to cool the machine
    end

    % evaluate
    e_sameNumVehs{i_priority} = EvaluationParl(results_full_path);
    % display progress
    count = count + 1;
    disp(['--------Progress ' num2str(count) '/' num2str(n_simulations) ': done--------'])
end

% get free flow speed, i.e., the speed that vehicles could travel if they are not influenced by others
% vehicles
if exist('Scenario','var')
    free_flow_speed = FreeFlowSpeed(scenario);
else
    free_flow_speed = FreeFlowSpeed();
end
disp('--------Finished--------')


%% Plot
set(0,'DefaultTextFontname', 'Times New Roman');
set(0,'DefaultAxesFontName', 'Times New Roman');
set(0,'defaultTextFontSize',11)
set(0,'defaultAxesFontSize',11)

speed_average_s = cellfun(@(c) c.average_speed_each_veh, e_sameNumVehs, 'UniformOutput', false);
grp_speed = cell2mat(arrayfun(@(i){i*ones(numel(speed_average_s{i}),1)},(1:numel(speed_average_s))')); 
speed_average = cellfun(@mean, speed_average_s);

fig_x = 6;     fig_y = 6; % [cm]
x_margin = 0;   y_margin = 0; 
fig_x_position = fig_x - x_margin;
fig_y_position = fig_y - y_margin;

file_name = 'evalPriorityAssignment_sameNumVehs';
fig = figure('Name',file_name);
set(fig, 'Units','centimeters', 'Position',[0 0 fig_x_position fig_y_position]/2)
set(fig, 'PaperUnits','centimeters','PaperSize',[fig_x fig_y],'PaperOrientation','portrait',...
    'PaperPosition', [-0.15 -0.15 fig_x_position+0.35 fig_y_position+0.3])

t_fig = tiledlayout(1,1,'Padding','compact','TileSpacing','compact');
X_string = {'STAC-based','Random','Constant'};
X_cat = categorical(X_string);
X_cat = reordercats(X_cat,X_string);

nexttile
% average speed
b1 = boxplot(vertcat(speed_average_s{:}),grp_speed,'Colors','k','OutlierSize',4,'Labels',X_string);
hAx=gca;                                
xtk=hAx.XTick;  
grid on
hold on
box on
% p = plot(xtk,horzcat(speed_average_s{:}));
s = scatter(xtk,speed_average,18,'filled','o','MarkerFaceColor',[0 0.4470 0.7410]);

free_flow_speed_i = free_flow_speed.free_flow_speed(free_flow_speed.sample_time==options.dt);
y_FFS = yline(free_flow_speed_i,'--b','LineWidth',0.6);

legend([s,y_FFS],{'Average','Free-flow'},'Location','southwest')
ylabel('$\overline{v}\:[m/s]$','Interpreter','latex')
xtickangle(0)

ylim([0.5 0.75])
% save fig
e_sameNumVehs{1}.save_fig(fig,file_name)
