%% Generate data: different allowed number of computation levels and different sample time
% prepare simulation options
options = OptionsMain;
options.consider_RSS = false;
options.is_sim_lab = true;
options.customResultName = '';
options.scenario_name = 'Commonroad';
options.trim_set = 9;
options.Hp = 5;
options.T_end = 5;
options.priority = 'STAC_priority';
options.isPB = true;
options.isParl = true;
options.isAllowInheritROW = false;
options.strategy_consider_veh_without_ROW = '3';
options.strategy_enter_lanelet_crossing_area = '4';
options.isSaveResult = true;
options.visu = [0,false];
options.is_eval = false;
options.visualize_reachable_set = false;
options.dt = 0.2;

% random_seed = RandStream('mt19937ar');
% options.veh_ids = sort(randsample(random_seed,1:40,options.amount),'ascend');
options.veh_ids = [11,18,19,20,26,27,29,31,32,40];
% options.veh_ids = [11,18,20,27,32,37,40,41];
options.amount = numel(options.veh_ids);

max_num_CLs_all = [1:1:5,options.amount];

e_CLs = cell(length(max_num_CLs_all),1);
n_simulations = numel(e_CLs);
count = 0;

for i_CL = 1:length(max_num_CLs_all)
    options.max_num_CLs = max_num_CLs_all(i_CL);

%     if options.max_num_CLs==options.amount
%         % Sequential trajectory planning: Vehicles are allowed to enter
%         % lanelet crossing areas
%         options.strategy_enter_lanelet_crossing_area = '1';
%     else
%         options.strategy_enter_lanelet_crossing_area = '1';
%     end
%     if i_CL==1
%         % without consideration of reachable set (may be computationally faster) 
%         options.isDealPredictionInconsistency = false;
%     else
%         options.isDealPredictionInconsistency = true;
%     end

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
        pause(5)
    end
    % data processing
    e_CLs{i_CL} = EvaluationParl(options);
    
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
%%
options.visu(1) = 1;
options.isSaveResult = 0;
options.max_num_CLs = 4;
[~,~,~] = main(options,scenario);

%% Plot
set(0,'DefaultTextFontname', 'Times New Roman');
set(0,'DefaultAxesFontName', 'Times New Roman');
set(0,'defaultTextFontSize',11)
set(0,'defaultAxesFontSize',11)

average_random_simulations = @(data) reshape(mean(reshape(data,random_times,[]),1),1,[]);

CT_graph_search = cellfun(@(c) c.runtime_graph_search, e_CLs);  
CT_graph_search_average = cellfun(@(c) c.runtime_graph_search_average, e_CLs);
CT_graph_search_pure_sequential = cellfun(@(c) c.runtime_graph_search_pure_sequential, e_CLs);  
CT_graph_search_pure_sequential_average = cellfun(@(c) c.runtime_graph_search_pure_sequential_average, e_CLs);
CT_graph_search(end+1) = CT_graph_search_pure_sequential(end);
CT_graph_search_average(end+1) = CT_graph_search_pure_sequential_average(end);


CT_total_max = cellfun(@(c) c.runtime_max, e_CLs);
% CT_total_max = average_random_simulations(CT_total_max);
CT_total_max = max(CT_total_max,[],1);

% number of coupling 
num_couplings = cellfun(@(c) c.num_couplings, e_CLs);
num_couplings = average_random_simulations(num_couplings);
num_couplings(end+1) = num_couplings(end);
num_couplings_ignored = cellfun(@(c) c.num_couplings_ignored, e_CLs);
num_couplings_ignored = average_random_simulations(num_couplings_ignored);
num_couplings_ignored(end+1) = num_couplings_ignored(end);
num_couplings_between_grps = cellfun(@(c) c.num_couplings_between_grps, e_CLs);
num_couplings_between_grps = average_random_simulations(num_couplings_between_grps);
num_couplings_between_grps(end+1) = num_couplings_between_grps(end);
num_couplings_between_grps_ignored = cellfun(@(c) c.num_couplings_between_grps_ignored, e_CLs);
num_couplings_between_grps_ignored = average_random_simulations(num_couplings_between_grps_ignored);
num_couplings_between_grps_ignored(end+1) = num_couplings_between_grps_ignored(end);

% maximum number of actual computation levels
CLs_num_max = cellfun(@(c) c.CLs_num_max, e_CLs);
CLs_num_max(end+1) = CLs_num_max(end);

% average speed
speed_average = cellfun(@(c) c.average_speed, e_CLs);
speed_average = average_random_simulations(speed_average);
speed_average(end+1) = speed_average(end);

plot_line_options = {};
plot_line_options{1}{1} = struct('LineWidth',0.6,'Color','#A2142F','LineStyle','-','Marker','*','MarkerSize',4); % average
plot_line_options{1}{2} = struct('LineWidth',0.6,'Color','#A2142F','LineStyle','-.','Marker','*','MarkerSize',4);% maximum
plot_line_options{2}{1} = struct('LineWidth',0.6,'Color','#7E2F8E','LineStyle','-','Marker','^','MarkerSize',4);
plot_line_options{2}{2} = struct('LineWidth',0.6,'Color','#7E2F8E','LineStyle','-.','Marker','^','MarkerSize',4);
plot_line_options{3}{1} = struct('LineWidth',0.6,'Color','#0072BD','LineStyle','-','Marker','o','MarkerSize',4);
plot_line_options{3}{2} = struct('LineWidth',0.6,'Color','#0072BD','LineStyle','-.','Marker','o','MarkerSize',4);
plot_line_options{4}{1} = struct('LineWidth',0.6,'Color','#D95319','LineStyle','-','Marker','v','MarkerSize',4);
plot_line_options{4}{2} = struct('LineWidth',0.6,'Color','#D95319','LineStyle','-.','Marker','v','MarkerSize',4);
plot_line_options{5}{1} = struct('LineWidth',0.6,'Color','#EDB120','LineStyle','-','Marker','s','MarkerSize',4);
plot_line_options{5}{2} = struct('LineWidth',0.6,'Color','#EDB120','LineStyle','-.','Marker','s','MarkerSize',4);
plot_line_options{6}{1} = struct('LineWidth',0.6,'Color','#77AC30','LineStyle','-','Marker','d','MarkerSize',4);
plot_line_options{6}{2} = struct('LineWidth',0.6,'Color','#77AC30','LineStyle','-.','Marker','d','MarkerSize',4);

fig_x = 14;     fig_y = 12; % [cm]
x_margin = 0;   y_margin = 0; 
fig_x_position = fig_x - 2*x_margin;
fig_y_position = fig_y - 2*y_margin;

file_name = 'evalCLs';
fig = figure('Name',file_name);
set(fig, 'Units','centimeters', 'Position',[0 0 fig_x_position fig_y_position]/2)
set(fig, 'PaperUnits','centimeters','PaperSize',[fig_x fig_y],'PaperOrientation','portrait',...
    'PaperPosition', [0 0 fig_x_position-0.1 fig_y_position])

t_fig = tiledlayout(2,2,'Padding','tight','TileSpacing','tight');

% X_string = {'Parallel B','Mix A','Mix B','Mix C','Sequential'};
X_string = {'Parallel','$n_{CLs}=3$','$n_{CLs}=5$','Bassam','Sequential'};
X_cat = categorical(X_string);
X_cat = reordercats(X_cat,X_string);
X_cat = categorical(max_num_CLs_all)
X_cat(end+1) = 'Sequential';
% plot computation time
nexttile(1,[1,1])
grid on
hold on
clear p
p(1) = plot(X_cat,CT_graph_search,plot_line_options{3}{2});
p(2) = plot(X_cat,CT_graph_search_average,plot_line_options{3}{1});
ylim([0,0.2])
legend(p,{'Maximum','Average'},'Location','best');

xlabel({'(a) Trajectory planning time.'},'Interpreter','latex');
ylabel('$t_c\:[s]$','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex');
set(gca, 'XTickLabel','')

% average number of couplings
nexttile(2,[1,1])
grid on
hold on
clear c
c(1) = plot(X_cat,num_couplings,plot_line_options{1}{1});
c(2) = plot(X_cat,num_couplings_ignored,plot_line_options{1}{2});
c(3) = plot(X_cat,num_couplings_between_grps,plot_line_options{2}{1});
c(4) = plot(X_cat,num_couplings_between_grps_ignored,plot_line_options{2}{2});
legend(c,{'Total','Decoupled (total)','Cross-group','Decoupled (c.-g.)'},'FontSize',9,'Location','best');
ylim([0 20])
xlabel({'(b) Number of couplings.'},'Interpreter','latex')
ylabel('$\overline{n}_{c}$','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex');
set(gca, 'XTickLabel','')


% maximum allowed and actual number of computation levels
nexttile(3)
grid on 
hold on
p_CLs(1) = plot(X_cat,[max_num_CLs_all,max_num_CLs_all(end)],plot_line_options{1}{1});
p_CLs(2) = plot(X_cat,CLs_num_max,plot_line_options{2}{1});
legend(p_CLs,{'Allowed','Actual'},'FontSize',9,'Location','northwest')
xlabel({'Trajectory planning type','(c) Computation levels.'},'Interpreter','latex');
ylabel('$n_{CLs},n_{CLs}^{act.}$','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex');
xtickangle(0)

% average speed
nexttile(4)
grid on
hold on
clear p_speed
b1 = bar(X_cat,speed_average);
xtips1 = b1(1).XEndPoints;
ytips1 = b1(1).YEndPoints;
labels1 = string(round(b1(1).YData,3));
text(xtips1,ytips1,labels1,'HorizontalAlignment','center','VerticalAlignment','bottom','FontSize',9)

free_flow_speed_i = free_flow_speed.free_flow_speed(free_flow_speed.sample_time==options.dt);
FFS = yline(free_flow_speed_i,'--b','LineWidth',0.6);
legend(FFS,{'Free-flow'},'FontSize',9,'Location','northeast','Interpreter','latex')

xlabel({'Trajectory planning type','(d) Average speed.'},'Interpreter','latex')
ylabel('$\overline{v}\:[m/s]$','Interpreter','latex')
ylim([0,1])
set(gca,'TickLabelInterpreter','latex');
xtickangle(0)

% save fig
e_CLs{1}.save_fig(fig,file_name)
