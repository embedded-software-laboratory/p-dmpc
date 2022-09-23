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
options.priority = 'STAC_priority';
options.isPB = true;
options.isParl = true;
options.isAllowInheritROW = false;
options.isSaveResult = true;
options.isSaveResultReduced = true;
options.visu = [false,false];
options.is_eval = false;
options.visualize_reachable_set = false;
options.strategy_consider_veh_without_ROW = '3';
options.strategy_enter_lanelet_crossing_area = '1';
nVeh_s = 20:1:30;
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
            e_predictionInconsistency{(i-1)*random_times+iRandom,iVeh} = EvaluationParl(options,[0,options.T_end]);
    
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

is_deadlock = cellfun(@(c) c.is_deadlock, e_predictionInconsistency)
% options for plot 
plot_options_considerPI = struct('LineWidth',.6,'Marker','*','MarkerSize',4);
plot_options_notConsiderPI = struct('LineWidth',.6,'Marker','o','MarkerSize',4);

fig_x = 9;     fig_y = 6; % [cm]
x_margin = 0;   y_margin = 0; 
fig_x_position = fig_x - 2*x_margin;
fig_y_position = fig_y - 2*y_margin;

file_name = 'evalPredictionInconsistency';
fig = figure('Name',file_name);
set(fig, 'Units','centimeters', 'Position',[0 0 fig_x_position fig_y_position]/2)
set(fig, 'PaperUnits','centimeters','PaperSize',[fig_x fig_y],'PaperOrientation','portrait',...
    'PaperPosition', [0 -0.04 fig_x_position-0.1 fig_y_position+0.05])

t_fig = tiledlayout(1,1,'Padding','tight','TileSpacing','none');

% total run time
nexttile
clear p
p(1) = plot(nVeh_s,t_total(1,:),plot_options_considerPI);
hold on 
p(2) = plot(nVeh_s,t_total(2,:),plot_options_notConsiderPI);
legend(p,{'Consider reachable sets','Consider previous trajectories'},'Location','northeast')
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

