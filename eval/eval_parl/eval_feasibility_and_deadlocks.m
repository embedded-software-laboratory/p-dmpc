%% Generate data: strategieis to improve feasibilities
    % 1. Higher-priority vehicles consider emergency braking maneuver of
    % lowr-priority vehicles
    % 2. 
strategy_feasibility_deadlock = {{'3','4'},{'3','1'},{'1','4'},{'1','1'}};
nVeh_s = 40:-2:30;

e_feasibility_differentNumVehs = cell(length(strategy_feasibility_deadlock),length(nVeh_s));

% prepare simulation options
options = OptionsMain;
options.consider_RSS = false;
options.is_sim_lab = true;
options.customResultName = '';
options.scenario = 'Commonroad';
options.trim_set = 9;
options.Hp = 5;

options.T_end = 300;
options.dt = 0.2;
options.max_num_CLs = 4;
options.priority = 'right_of_way_priority';
options.isPB = true;
options.isParl = true;
options.isAllowInheritROW = false;
options.isSaveResult = true;
options.visu = [false,false];
options.is_eval = false;
options.visualize_reachable_set = false;

n_simulations = numel(e_feasibility_differentNumVehs);
count = 0;

for i = 1:length(strategy_feasibility_deadlock)
    for j = 1:length(nVeh_s)
        options.strategy_consider_veh_without_ROW = strategy_feasibility_deadlock{i}{1};
        options.strategy_enter_lanelet_crossing_area = strategy_feasibility_deadlock{i}{2};
        options.amount = nVeh_s(j);
    
        full_path = FileNameConstructor.get_results_full_path(options);
        if isfile(full_path)
            disp('File already exists.')
        else
            % run simulation
            if exist('options','var') && exist('scenario','var')
                [~,~,~] = main(options,scenario);
            else
                [~,scenario,~] = main(options);
            end
            pause(10) 
        end
    
        % data processing
        e_feasibility_differentNumVehs{i,j} = EvaluationParl(options);
    
        % display progress
        count = count + 1;
        disp(['--------Progress ' num2str(count) '/' num2str(n_simulations) ': done--------'])
    end
end
disp('--------Finished--------')

%% Plot
set(0,'DefaultTextFontname', 'Times New Roman');
set(0,'DefaultAxesFontName', 'Times New Roman');
set(0,'defaultTextFontSize',7)
set(0,'defaultAxesFontSize',7)

max_runtime_s = cellfun(@(c) c.t_total, e_feasibility_differentNumVehs);
% speed_average_s = cellfun(@(c) c.average_speed, e_feasibility_differentNumVehs);
fallback_rate_s = cellfun(@(c) c.fallback_rate, e_feasibility_differentNumVehs);
% speed_sum_s = cellfun(@(c) c.sum_speeds_all_vehicles, e_feasibility_differentNumVehs);

fig_x = 6;     fig_y = 4; % [cm]
x_margin = 0;   y_margin = 0; 
fig_x_position = fig_x - 2*x_margin;
fig_y_position = fig_y - 2*y_margin;

file_name = 'evalFeasibilityDeadlock_differentNumVehs';
fig = figure('Name',file_name);
set(fig, 'Units','centimeters', 'Position',[0 0 fig_x_position fig_y_position]/2)
set(fig, 'PaperUnits','centimeters','PaperSize',[fig_x fig_y],'PaperOrientation','portrait',...
    'PaperPosition', [x_margin y_margin fig_x_position fig_y_position])

t_fig = tiledlayout(1,2,'Padding','none','TileSpacing','compact');
X_string = {'Before','After'};
X_cat = categorical(X_string);

nexttile
% 1. max. number of vehicles
b1 = bar(X_cat,nVeh_s);
grid on
xtips1 = b1(1).XEndPoints;
ytips1 = b1(1).YEndPoints;
labels1 = string(b1(1).YData);
text(xtips1,ytips1,labels1,'HorizontalAlignment','center','VerticalAlignment','bottom')
ylim([0 38])
xlabel({'(a). Maximum allowed number of vehicles.'})
ylabel('$n_{veh}^{max}$','Interpreter','latex')

nexttile
% 2. speed
b2 = bar(X_cat,[speed_average_s,speed_sum_s]);
grid on
xtips1 = b2(1).XEndPoints;
ytips1 = b2(1).YEndPoints;
labels1 = string(round(b2(1).YData,2));
text(xtips1-0.03,ytips1,labels1,'HorizontalAlignment','center','VerticalAlignment','bottom')
xtips2 = b2(2).XEndPoints;
ytips2 = b2(2).YEndPoints;
labels2 = string(round(b2(2).YData,2));
text(xtips2,ytips2,labels2,'HorizontalAlignment','center','VerticalAlignment','bottom')
xlabel('(b). Average speed and sum of speeds.')
ylabel('$\overline{v}\:[m/s]$','Interpreter','latex')
legend({'Average','Sum'},'Location','best')
ylim([0 27])

% save fig
e_feasibility_differentNumVehs{1}.save_fig(fig,file_name)
%% Generate data: strategieis to improve feasibilities (same number of vehicles)
    % 1. Higher-priority vehicles consider emergency braking maneuver of
    % lowr-priority vehicles
    % 2. 
strategy_feasibility_deadlock = {{'3','1'},{'1','1'}};
nVeh_s = [18,18];

e_feasibility_sameNumVehs = cell(1,length(strategy_feasibility_deadlock));

% prepare simulation options
options = OptionsMain;
options.consider_RSS = false;
options.is_sim_lab = true;
options.customResultName = '';
options.scenario = 'Commonroad';
options.trim_set = 9;
options.Hp = 5;

options.T_end = 20;
options.dt = 0.2;
options.max_num_CLs = 3;
options.priority = 'right_of_way_priority';
options.isPB = true;
options.isParl = true;
options.isAllowInheritROW = true;
options.isSaveResult = true;
options.visu = [false,false];
options.is_eval = false;
options.visualize_reachable_set = false;
options.amount = nVeh_s(end);

for i = 1:length(strategy_feasibility_deadlock)
    options.strategy_consider_veh_without_ROW = strategy_feasibility_deadlock{i}{1};
    options.strategy_enter_lanelet_crossing_area = strategy_feasibility_deadlock{i}{2};
    
    full_path = FileNameConstructor.get_results_full_path(options);
    if isfile(full_path)
        disp('File already exists.')
    else
        % run simulation
        if exist('options','var') && exist('scenario','var')
            [~,~,~] = main(options,scenario);
        else
            [~,scenario,~] = main(options);
        end
        disp('Pause to cool the CPU...')
        pause(3) 
    end

    % data processing
    e_feasibility_sameNumVehs{i} = EvaluationParl(options);

    disp([num2str(i) ': done.'])
end
disp('--------Finished--------')

%% Plot
set(0,'DefaultTextFontname', 'Times New Roman');
set(0,'DefaultAxesFontName', 'Times New Roman');
set(0,'defaultTextFontSize',9)
set(0,'defaultAxesFontSize',9)

nVeh_s = cellfun(@(c) c.nVeh, e_feasibility_sameNumVehs);
speed_average_s = cellfun(@(c) c.average_speed, e_feasibility_sameNumVehs);
speed_sum_s = cellfun(@(c) c.sum_speeds_all_vehicles, e_feasibility_sameNumVehs);

runtime_s = cellfun(@(c) c.runtime_total_per_step, e_feasibility_sameNumVehs, 'UniformOutput',false);
grp_runtime = cell2mat(arrayfun(@(i){i*ones(numel(runtime_s{i}),1)},(1:numel(runtime_s))')); 

fig_x = 11.5;     fig_y = 5; % [cm]
x_margin = 0;   y_margin = 0; 
fig_x_position = fig_x - 2*x_margin;
fig_y_position = fig_y - 2*y_margin;

file_name = 'evalFeasibilityDeadlock_sameNumVehs';
fig = figure('Name',file_name);
set(fig, 'Units','centimeters', 'Position',[0 0 fig_x_position fig_y_position]/2)
set(fig, 'PaperUnits','centimeters','PaperSize',[fig_x fig_y],'PaperOrientation','portrait',...
    'PaperPosition', [x_margin y_margin fig_x_position fig_y_position])

t_fig = tiledlayout(1,4,'Padding','none','TileSpacing','compact');
X_string = {'With','Without'};
X_cat = categorical(X_string);

nexttile
% 1. max. number of vehicles
b1 = bar(X_cat,nVeh_s(1:2));
grid on
xtips1 = b1(1).XEndPoints;
ytips1 = b1(1).YEndPoints;
labels1 = string(b1(1).YData);
text(xtips1,ytips1,labels1,'HorizontalAlignment','center','VerticalAlignment','bottom')
ylim([0 38])
xlabel({'(a). Max. allowed number of vehicles.'})
ylabel('$n_{veh}^{max}$','Interpreter','latex')

nexttile
% 2. speed
b2 = bar(X_cat,[speed_average_s(2:3),speed_sum_s(2:3)]);
grid on
xtips1 = b2(1).XEndPoints;
ytips1 = b2(1).YEndPoints;
labels1 = string(round(b2(1).YData,2));
text(xtips1-0.03,ytips1,labels1,'HorizontalAlignment','center','VerticalAlignment','bottom')
xtips2 = b2(2).XEndPoints;
ytips2 = b2(2).YEndPoints;
labels2 = string(round(b2(2).YData,2));
text(xtips2,ytips2,labels2,'HorizontalAlignment','center','VerticalAlignment','bottom')
xlabel('(b). Average speed and sum of speeds.')
ylabel('$\overline{v}\:[m/s]$','Interpreter','latex')
legend({'Average','Sum'},'Location','best')
ylim([0 20])

nexttile
% collision-free runtime (simulation ends at time step 11000. Totally 59
% time steps involve fallbacks. Assume half of vehicles take fallback at
% those time steps in average) 
collision_free_runtime = [11000,154]*options.dt./60;
fallback_rate_s = [59/11000*0.5,e_feasibility_sameNumVehs{end}.fallback_rate]*100;
b3 = bar(X_cat,collision_free_runtime);
grid on
xtips3 = b3(1).XEndPoints;
ytips3 = b3(1).YEndPoints;
labels3 = string(round(b3(1).YData,2));
text(xtips3,ytips3,labels3,'HorizontalAlignment','center','VerticalAlignment','bottom')
ylim([0 42])
xlabel({'$n_{veh}$','(c). Maximum runtime.'},'Interpreter','latex')
ylabel('$t_{max}\:[s]$','Interpreter','latex')

nexttile
% fallback rate
plot(X_cat,fallback_rate_s,'-*k','LineWidth',0.5,'MarkerSize',4)
ylabel('Fallback rate $[\%]$','Interpreter','latex')
xlabel(t_fig,'Improvement of Feasibility','FontSize',9,'FontName','Times New Roman')

% save fig
e_feasibility_sameNumVehs{1}.save_fig(fig,file_name)


