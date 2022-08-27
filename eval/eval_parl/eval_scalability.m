%% evaluate the scalability of the coupling determination algorithm, vehicle grouping algorithm
nVeh_s = 1:2:40;

% prepare simulation options
options = OptionsMain;
options.consider_RSS = false;
options.is_sim_lab = true;
options.customResultName = '';
options.scenario_name = 'Commonroad';
options.trim_set = 9;
options.Hp = 5;

options.T_end = 10;
options.dt = 0.2;
options.max_num_CLs = 4;
options.priority = 'right_of_way_priority';
options.isPB = true;
options.isParl = true;
options.isAllowInheritROW = false;
options.isSaveResult = true;
options.isSaveResultReduced = false;
options.visu = [false,false];
options.is_eval = false;
options.visualize_reachable_set = false;
options.strategy_consider_veh_without_ROW = '3';
options.strategy_enter_lanelet_crossing_area = '4';

% Random choose different vehicles three times
random_times = 1;
e_scalability = cell(random_times,length(nVeh_s));
n_simulations = numel(e_scalability);
count = 0;

for iVeh = 1:length(nVeh_s)
    options.amount = nVeh_s(iVeh);

    random_seed = RandStream('mt19937ar');
    for iRandom=1:random_times
        options.random_idx =  iRandom;
        options.veh_ids = sort(randsample(random_seed,1:40,options.amount),'ascend');

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
        end
        % data processing
        e_scalability{iRandom,iVeh} = EvaluationParl(options);

        % display progress
        count = count + 1;
        disp(['--------Progress ' num2str(count) '/' num2str(n_simulations) ': done--------'])
    end
end
disp('--------Finished--------')

%% Plot
set(0,'DefaultTextFontname', 'Times New Roman');
set(0,'DefaultAxesFontName', 'Times New Roman');
set(0,'defaultTextFontSize',11)
set(0,'defaultAxesFontSize',11)

average_random_simulations = @(data) reshape(mean(reshape(data,random_times,[]),1),1,[]);

% computation time of different parts
CT_group_vehs = cellfun(@(c) c.runtime_group_vehs, e_scalability);
% average of the random simulations
CT_group_vehs = average_random_simulations(CT_group_vehs);
CT_group_vehs_average = cellfun(@(c) c.runtime_group_vehs_average, e_scalability);
CT_group_vehs_average = average_random_simulations(CT_group_vehs_average);

CT_determine_couplings = cellfun(@(c) c.runtime_determine_couplings, e_scalability);
CT_determine_couplings = average_random_simulations(CT_determine_couplings);
CT_determine_couplings_average = cellfun(@(c) c.runtime_determine_couplings_average, e_scalability);
CT_determine_couplings_average = average_random_simulations(CT_determine_couplings_average);

CT_assign_priority = cellfun(@(c) c.runtime_assign_priority, e_scalability);
CT_assign_priority = average_random_simulations(CT_assign_priority);
CT_assign_priority_average = cellfun(@(c) c.runtime_assign_priority_average, e_scalability);
CT_assign_priority_average = average_random_simulations(CT_assign_priority_average);

CT_graph_search = cellfun(@(c) c.runtime_graph_search, e_scalability);
CT_graph_search = average_random_simulations(CT_graph_search);
CT_graph_search_average = cellfun(@(c) c.runtime_graph_search_average, e_scalability);
CT_graph_search_average = average_random_simulations(CT_graph_search_average);

% total runtime
t_total = cellfun(@(c) c.t_total, e_scalability);
t_total = average_random_simulations(t_total);
CT_total = cellfun(@(c) c.runtime_max, e_scalability);
CT_total = average_random_simulations(CT_total);
CT_total_average = cellfun(@(c) c.runtime_total_average, e_scalability);
CT_total_average = average_random_simulations(CT_total_average);

% number of coupling 
num_couplings = cellfun(@(c) c.num_couplings, e_scalability);
num_couplings = average_random_simulations(num_couplings);
CLs_num_max = cellfun(@(c) c.CLs_num_max, e_scalability);
CLs_num_max = average_random_simulations(CLs_num_max);

fallback_rate = cellfun(@(c) c.fallback_rate*100, e_scalability);
fallback_rate = average_random_simulations(fallback_rate);

% average speed
speed_average = cellfun(@(c) c.average_speed, e_scalability);
speed_average = average_random_simulations(speed_average);

% options for plot 
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

fig_x = 16;     fig_y = 10; % [cm]
x_margin = 0;   y_margin = 0; 
fig_x_position = fig_x - 2*x_margin;
fig_y_position = fig_y - 2*y_margin;

file_name = 'evalScalability';
fig = figure('Name',file_name);
set(fig, 'Units','centimeters', 'Position',[0 0 fig_x_position fig_y_position]/2)
set(fig, 'PaperUnits','centimeters','PaperSize',[fig_x fig_y],'PaperOrientation','portrait',...
    'PaperPosition', [x_margin y_margin fig_x_position fig_y_position])

t_fig = tiledlayout(1,2,'Padding','compact','TileSpacing','compact');

% total run time
nexttile
clear p
hold on 
grid on
p(1) = plot(nVeh_s,CT_total,plot_line_options{1}{2});
p(2) = plot(nVeh_s,CT_total_average,plot_line_options{1}{1});
p(3) = plot(nVeh_s,CT_assign_priority,plot_line_options{5}{2});
p(4) = plot(nVeh_s,CT_assign_priority_average,plot_line_options{5}{1});
p(5) = plot(nVeh_s,CT_determine_couplings,plot_line_options{2}{2});
p(6) = plot(nVeh_s,CT_determine_couplings_average,plot_line_options{2}{1});
p(7) = plot(nVeh_s,CT_graph_search,plot_line_options{3}{2});
p(8) = plot(nVeh_s,CT_graph_search_average,plot_line_options{3}{1});
p(9) = plot(nVeh_s,CT_group_vehs,plot_line_options{4}{2});
p(10) = plot(nVeh_s,CT_group_vehs_average,plot_line_options{4}{1});

xlabel({'$n_{veh}$','(a). Computation time per step.'},'Interpreter','latex')
ylabel('$t_c\:[s]$','Interpreter','latex')
legend(p,{'Total (max.)','Total (avg.)','Vehicle prioritization (max.)','Vehicle prioritization (avg.)','Coupling determination (max.)','Coupling determination (avg.)','Trajectory planning (max.)','Trajectory planning (avg.)','Vehicle grouping (max.)','Vehicle grouping (avg.)'}, ...
    'Orientation','horizontal','NumColumns',1,'Location','northwest')
xlim([min(nVeh_s) max(nVeh_s)])
ylim([0 0.25])
xticks(nVeh_s)

% number of couplings and actual number of computation levels
nexttile
hold on 
grid on
p_b(1) = plot(nVeh_s,num_couplings,plot_line_options{1}{1});
p_b(2) = plot(nVeh_s,CLs_num_max,plot_line_options{2}{1});
legend(p_b,{'Average number of couplings','Actual number of computation levels'})
xlabel({'$n_{veh}$','(b). Number of couplings and computation levels.'},'Interpreter','latex')
ylabel('$\overline{n}_c,n_{CLs}^{act.}$','Interpreter','latex')
xlim([min(nVeh_s) max(nVeh_s)])
y_max = 70;
ylim([0 y_max])
xticks(nVeh_s)
yticks(0:5:y_max)
% xlabel(t_fig,'Compare different fallback strategies','FontSize',9,'FontName','Times New Roman')
% save fig
e_scalability{1}.save_fig(fig,file_name)