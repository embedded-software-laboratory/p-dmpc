%% Generate data: different allowed number of computation levels and different sample time
% prepare simulation options
options = OptionsMain;
options.consider_RSS = false;
options.is_sim_lab = true;
options.customResultName = '';
options.scenario = 'Commonroad';
options.trim_set = 9;
options.Hp = 6;
options.amount = 20;
options.T_end = 10;
options.priority = 'right_of_way_priority';
options.isPB = true;
options.isParl = true;
options.isAllowInheritROW = true;
options.strategy_consider_veh_without_ROW = '3';
options.isSaveResult = true;
options.visu = [false,false];
options.is_eval = false;
options.visualize_reachable_set = false;
options.dt = 0.2;

max_num_CLs_all = [1:ceil(options.amount/2),options.amount];

% Random choose different vehicles three times
random_times = 1;
e_CLs = cell(random_times,length(max_num_CLs_all));
n_simulations = numel(e_CLs);
count = 0;

for i_CL = 1:length(max_num_CLs_all)
    options.max_num_CLs = max_num_CLs_all(i_CL);

    if options.max_num_CLs==options.amount
        % Sequential trajectory planning: Vehicles are allowed to enter
        % lanelet crossing areas
        options.strategy_enter_lanelet_crossing_area = '1';
    else
        options.strategy_enter_lanelet_crossing_area = '4';
    end

    random_seed = RandStream('mt19937ar');
    for iRandom=1:random_times
        options.random_idx =  iRandom;
        options.veh_ids = sort(randsample(random_seed,9:40,options.amount),'ascend');

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
        e_CLs{iRandom,i_CL} = EvaluationParl(options);
        
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
disp('--------Finished--------')

%% Plot
set(0,'DefaultTextFontname', 'Times New Roman');
set(0,'DefaultAxesFontName', 'Times New Roman');
set(0,'defaultTextFontSize',7)
set(0,'defaultAxesFontSize',7)

average_random_simulations = @(data) reshape(mean(reshape(data,random_times,[]),1),1,[]);

% computation time of different parts
CT_group_vehs = cellfun(@(c) c.runtime_group_vehs, e_CLs);
% CT_group_vehs = average_random_simulations(CT_group_vehs); % average of the random simulations
CT_group_vehs = max(CT_group_vehs,[],1);

CT_determine_couplings = cellfun(@(c) c.runtime_determine_couplings, e_CLs);
% CT_determine_couplings = average_random_simulations(CT_determine_couplings);
CT_determine_couplings = max(CT_determine_couplings,[],1);

CT_determine_couplings_average = cellfun(@(c) c.runtime_determine_couplings_average, e_CLs);
% CT_determine_couplings = average_random_simulations(CT_determine_couplings);
CT_determine_couplings_average = max(CT_determine_couplings_average,[],1);

CT_assign_priority = cellfun(@(c) c.runtime_assign_priority, e_CLs);
% CT_assign_priority = average_random_simulations(CT_assign_priority);
CT_assign_priority = max(CT_assign_priority,[],1);

CT_graph_search = cellfun(@(c) c.runtime_graph_search, e_CLs);
% CT_graph_search = average_random_simulations(CT_graph_search);
CT_graph_search = max(CT_graph_search,[],1);
CT_graph_search_average = cellfun(@(c) c.runtime_graph_search_average, e_CLs);
% CT_graph_search = average_random_simulations(CT_graph_search);
CT_graph_search_average = max(CT_graph_search_average,[],1);

CT_total_each_step_tmp = cellfun(@(c) c.runtime_total_per_step, e_CLs, 'UniformOutput',false);
CT_total_each_step = cell(1,length(max_num_CLs_all));
for i = 1:size(CT_total_each_step_tmp,2)
    CT_total_each_step{i} = vertcat(CT_total_each_step_tmp{:,i});
end
grp_CT_total_each_step = cell2mat(arrayfun(@(i){i*ones(numel(CT_total_each_step{i}),1)},(1:numel(CT_total_each_step))')); 

CT_total_average = cellfun(@mean, CT_total_each_step);

CT_total_max = cellfun(@(c) c.runtime_max, e_CLs);
% CT_total_max = average_random_simulations(CT_total_max);
CT_total_max = max(CT_total_max,[],1);

% total runtime
t_total = cellfun(@(c) c.t_total, e_CLs);
t_total = average_random_simulations(t_total);

% number of coupling 
num_couplings = cellfun(@(c) c.num_couplings, e_CLs);
num_couplings = average_random_simulations(num_couplings);
num_couplings_ignored = cellfun(@(c) c.num_couplings_ignored, e_CLs);
num_couplings_ignored = average_random_simulations(num_couplings_ignored);
num_couplings_between_grps = cellfun(@(c) c.num_couplings_between_grps, e_CLs);
num_couplings_between_grps = average_random_simulations(num_couplings_between_grps);
num_couplings_between_grps_ignored = cellfun(@(c) c.num_couplings_between_grps_ignored, e_CLs);
num_couplings_between_grps_ignored = average_random_simulations(num_couplings_between_grps_ignored);

% maximum numbero of actual computation levels
CLs_num_max = cellfun(@(c) c.CLs_num_max, e_CLs);
CLs_num_max = max(CLs_num_max,[],1);
% fallback rate
fallback_rate = cellfun(@(c) c.fallback_rate*100, e_CLs);
fallback_rate = average_random_simulations(fallback_rate);

% average speed
speed_average = cellfun(@(c) c.average_speed, e_CLs);
speed_average = average_random_simulations(speed_average);
speed_max = cellfun(@(c) c.max_speed, e_CLs);
speed_max = average_random_simulations(speed_max);
speed_min = cellfun(@(c) c.min_speed, e_CLs);
speed_min = average_random_simulations(speed_min);


speed_each_veh_tmp = cellfun(@(c) c.average_speed_each_veh, e_CLs, 'UniformOutput',false);
speed_each_veh = cell(1,length(max_num_CLs_all));
for i = 1:size(speed_each_veh_tmp,2)
    speed_each_veh{i} = vertcat(speed_each_veh_tmp{:,i});
end
grp_speed_each_veh = cell2mat(arrayfun(@(i){i*ones(numel(speed_each_veh{i}),1)},(1:numel(speed_each_veh))')); 

plot_line_options = {};
plot_line_options{1}{1} = struct('LineWidth',0.5,'Color','#A2142F','LineStyle','-','Marker','*','MarkerSize',4); % average
plot_line_options{1}{2} = struct('LineWidth',0.5,'Color','#A2142F','LineStyle','-.','Marker','*','MarkerSize',4);% maximum
plot_line_options{2}{1} = struct('LineWidth',0.5,'Color','#7E2F8E','LineStyle','-','Marker','^','MarkerSize',4);
plot_line_options{2}{2} = struct('LineWidth',0.5,'Color','#7E2F8E','LineStyle','-.','Marker','^','MarkerSize',4);
plot_line_options{3}{1} = struct('LineWidth',0.5,'Color','#0072BD','LineStyle','-','Marker','o','MarkerSize',4);
plot_line_options{3}{2} = struct('LineWidth',0.5,'Color','#0072BD','LineStyle','-.','Marker','o','MarkerSize',4);
plot_line_options{4}{1} = struct('LineWidth',0.5,'Color','#D95319','LineStyle','-','Marker','v','MarkerSize',4);
plot_line_options{4}{2} = struct('LineWidth',0.5,'Color','#D95319','LineStyle','-.','Marker','v','MarkerSize',4);
plot_line_options{5}{1} = struct('LineWidth',0.5,'Color','#EDB120','LineStyle','-','Marker','s','MarkerSize',4);
plot_line_options{5}{2} = struct('LineWidth',0.5,'Color','#EDB120','LineStyle','-.','Marker','s','MarkerSize',4);
plot_line_options{6}{1} = struct('LineWidth',0.5,'Color','#77AC30','LineStyle','-','Marker','d','MarkerSize',4);
plot_line_options{6}{2} = struct('LineWidth',0.5,'Color','#77AC30','LineStyle','-.','Marker','d','MarkerSize',4);

fig_x = 10;     fig_y = 13; % [cm]
x_margin = 0;   y_margin = 0; 
fig_x_position = fig_x - 2*x_margin;
fig_y_position = fig_y - 2*y_margin;

file_name = 'evalCLs';
fig = figure('Name',file_name);
set(fig, 'Units','centimeters', 'Position',[0 0 fig_x_position fig_y_position]/2)
set(fig, 'PaperUnits','centimeters','PaperSize',[fig_x fig_y],'PaperOrientation','portrait',...
    'PaperPosition', [x_margin y_margin fig_x_position fig_y_position])

t_fig = tiledlayout(3,2,'Padding','compact','TileSpacing','compact');

% plot computation time
nexttile(1,[2,1])
grid on
hold on
clear p
p(1) = plot(categorical(max_num_CLs_all),CT_total_max,plot_line_options{1}{2});
p(2) = plot(categorical(max_num_CLs_all),CT_total_average,plot_line_options{1}{1});
p(3) = plot(categorical(max_num_CLs_all),CT_graph_search,plot_line_options{2}{2});
p(4) = plot(categorical(max_num_CLs_all),CT_graph_search_average,plot_line_options{2}{1});
% p(4) = plot(categorical(max_num_CLs_all),CT_assign_priority,plot_line_options(4));
p(5) = plot(categorical(max_num_CLs_all),CT_determine_couplings,plot_line_options{3}{2});
p(6) = plot(categorical(max_num_CLs_all),CT_determine_couplings_average,plot_line_options{3}{1});
% p(6) = plot(categorical(max_num_CLs_all),CT_group_vehs,plot_line_options(6));
% plot line indicating the sample time 
p(7) = yline(options.dt,'--b','LineWidth',0.5);
ylim([0,0.3])
legend(p,{'Total (max.)','Total (avg.)','Trajectory (max.)','Trajectory (avg.)', ...
    'Coupling (max.)','Coupling (avg.)','Sample time'},'FontSize',7,'Location','best');

xlabel({'(a) Computation time per step.'},'Interpreter','latex');
ylabel('$t_c\:[s]$','Interpreter','latex')

% average number of couplings
nexttile(2,[2,1])
grid on
hold on
clear c
c(1) = plot(categorical(max_num_CLs_all),num_couplings,plot_line_options{1}{1});
c(2) = plot(categorical(max_num_CLs_all),num_couplings_ignored,plot_line_options{1}{2});
c(3) = plot(categorical(max_num_CLs_all),num_couplings_between_grps,plot_line_options{2}{1});
c(4) = plot(categorical(max_num_CLs_all),num_couplings_between_grps_ignored,plot_line_options{2}{2});
legend(c,{'Total','Ignored (total)','Between groups','Ignored (between g.)'},'FontSize',7,'Location','south')
xlabel({'(b) Number of couplings.'},'Interpreter','latex')
ylabel('$\overline{n}_{c}$','Interpreter','latex')

% maximum allowed and actual number of computation levels
nexttile(5)
grid on 
hold on
p_CLs(1) = plot(categorical(max_num_CLs_all),max_num_CLs_all,plot_line_options{1}{1});
p_CLs(2) = plot(categorical(max_num_CLs_all),CLs_num_max,plot_line_options{2}{1});
legend(p_CLs,{'Allowed','Actual'},'FontSize',7,'Location','best')
xlabel({'$n_{CLs}$','(c) Computation levels.'},'Interpreter','latex');
ylabel('$n_{CLs}$','Interpreter','latex')

% average speed
nexttile(6)
grid on
hold on
clear p_speed
p_speed(1) = plot(categorical(max_num_CLs_all),speed_average,plot_line_options{1}{1});
free_flow_speed_i = free_flow_speed.free_flow_speed(free_flow_speed.sample_time==options.dt);
p_speed(2) = yline(free_flow_speed_i,'--b','LineWidth',0.5);
legend(p_speed,{'Average','Free-flow'},'FontSize',7,'Location','layout','Interpreter','latex')

xlabel({'$n_{CLs}$','(d) Average speed.'},'Interpreter','latex')
ylabel('$\overline{v}\:[m/s]$','Interpreter','latex')
ylim([0.55,0.75])



% plot fallback rate 
% nexttile
% grid on
% hold on
% plot(categorical(max_num_CLs_all),fallback_rate,'-*k','LineWidth',0.5,'MarkerSize',4) 
% ylim([0,3])
% xlabel({'$n_{CLs}$','(c) Fallback rate.'},'Interpreter','latex')
% ylabel('$\overline{p}_{FB}\:[\%]$','Interpreter','latex')

% save fig
e_CLs{1}.save_fig(fig,file_name)
