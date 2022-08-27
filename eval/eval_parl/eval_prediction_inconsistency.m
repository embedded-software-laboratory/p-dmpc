%% evaluate the strategy of avoiding the problem of prediction inconsistency
isDealPredictionInconsistency = [true;false];
% prepare simulation options
options = OptionsMain;
options.consider_RSS = false;
options.is_sim_lab = true;
options.customResultName = '';
options.scenario_name = 'Commonroad';
options.trim_set = 9;
options.Hp = 6;

options.T_end = 300;
options.dt = 0.2;
options.max_num_CLs = 1;
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
nVeh_s = 18:1:30;
% Random choose different vehicles three times
random_times = 3;
e_predictionInconsistency = cell(random_times*length(isDealPredictionInconsistency),length(nVeh_s));
n_simulations = numel(e_predictionInconsistency);
count = 0;

for i = 1:length(isDealPredictionInconsistency)
    options.isDealPredictionInconsistency = isDealPredictionInconsistency(i);
    for iVeh = 1:length(nVeh_s)
        options.amount = nVeh_s(iVeh);
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
            end
            % data processing
            e_predictionInconsistency{(i-1)*random_times+iRandom,iVeh} = EvaluationParl(options);
    
            % display progress
            count = count + 1;
            disp(['--------Progress ' num2str(count) '/' num2str(n_simulations) ': done--------'])
        end
    end
end
disp('--------Finished--------')
%% Plot
set(0,'DefaultTextFontname', 'Times New Roman');
set(0,'DefaultAxesFontName', 'Times New Roman');
set(0,'defaultTextFontSize',11)
set(0,'defaultAxesFontSize',11)

average_random_simulations = @(data) reshape(mean(reshape(data,random_times,[]),1),length(isDealPredictionInconsistency),[]);

% total runtime
t_total_tmp = cellfun(@(c) c.t_total, e_predictionInconsistency);
t_total = average_random_simulations(t_total_tmp);
t_total_normalized = t_total(1,:)./t_total(2,:);

fallback_rate_tmp = cellfun(@(c) c.fallback_rate*100, e_predictionInconsistency);
fallback_rate = average_random_simulations(fallback_rate_tmp);
% speed_sum_s = cellfun(@(c) c.sum_speeds_all_vehicles, e_feasibility_differentNumVehs);

speed_average_tmp = cellfun(@(c) c.average_speed, e_predictionInconsistency);
speed_average = average_random_simulations(speed_average_tmp);

speed_average_each_veh_tmp = cellfun(@(c) c.average_speed_each_veh, e_predictionInconsistency, 'UniformOutput',false);
speed_average_each_veh = cell(1,length(strategy_feasibility_deadlock));
for i = 1:size(speed_average_each_veh_tmp,2)
    speed_average_each_veh{i} = vertcat(speed_average_each_veh_tmp{:,i});
end
grp_speed_average_each_veh = cell2mat(arrayfun(@(i){i*ones(numel(speed_average_each_veh{i}),1)},(1:numel(speed_average_each_veh))')); 

CT_graph_search_tmp = cellfun(@(c) c.runtime_graph_search, e_predictionInconsistency);
CT_graph_search = average_random_simulations(CT_graph_search_tmp);
CT_total_max_tmp = cellfun(@(c) c.runtime_max, e_predictionInconsistency);
CT_total_max = average_random_simulations(CT_total_max_tmp);
CT_total_average_tmp = cellfun(@(c) c.runtime_total_average, e_predictionInconsistency);
CT_total_average = average_random_simulations(CT_total_average_tmp);
X_string = {'Both','FIS','DAS','None'};
X_cat = categorical(X_string);
X_cat = reordercats(X_cat,X_string);

% options for plot 
plot_options_considerPI = struct('LineWidth',.5,'Marker','*','MarkerSize',4);
plot_options_notConsiderPI = struct('LineWidth',.5,'Marker','o','MarkerSize',4);
plot_options_noFB = struct('LineWidth',.5,'Marker','square','MarkerSize',4);

fig_x = 8;     fig_y = 8; % [cm]
x_margin = 0;   y_margin = 0; 
fig_x_position = fig_x - 2*x_margin;
fig_y_position = fig_y - 2*y_margin;

file_name = 'evalPredictionInconsistency';
fig = figure('Name',file_name);
set(fig, 'Units','centimeters', 'Position',[0 0 fig_x_position fig_y_position]/2)
set(fig, 'PaperUnits','centimeters','PaperSize',[fig_x fig_y],'PaperOrientation','portrait',...
    'PaperPosition', [x_margin y_margin fig_x_position fig_y_position])

t_fig = tiledlayout(1,1,'Padding','compact','TileSpacing','tight');

% total run time
nexttile
clear p
p(1) = plot(nVeh_s,t_total(1,:),plot_options_considerPI);
hold on 
p(2) = plot(nVeh_s,t_total(2,:),plot_options_notConsiderPI);
legend(p,{'Consider reachable set','Consider one-step delayed trajectory'},'Location','northoutside')
grid on
xlabel({'$n_{veh}$'},'Interpreter','latex')
ylabel('$t_{max}\:[s]$','Interpreter','latex')
xlim([min(nVeh_s) max(nVeh_s)])
xticks(nVeh_s)

t_total_a = sum(t_total(1,:));
t_total_b = sum(t_total(2,:));
average_improvement = (t_total_a-t_total_b)/t_total_b;
disp(['Average improvement in maximum runtime: ' num2str(average_improvement*100) '%.'])
% save fig
e_predictionInconsistency{1}.save_fig(fig,file_name)

