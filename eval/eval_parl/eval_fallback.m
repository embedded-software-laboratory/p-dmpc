%% Generate data: local/global/no fallback
fallback_types = {'localFallback','globalFallback','noFallback'};
nVeh_s = 20:2:40;

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
options.max_num_CLs = 3;
options.priority = 'right_of_way_priority';
options.isPB = true;
options.isParl = true;
options.isAllowInheritROW = false;
options.isSaveResult = true;
options.isSaveResultReduced = true;
options.visu = [false,false];
options.is_eval = false;
options.visualize_reachable_set = false;
options.strategy_consider_veh_without_ROW = '3';
options.strategy_enter_lanelet_crossing_area = '4';

% Random choose different vehicles three times
random_times = 3;
e_fallback_strategies = cell(length(fallback_types)*random_times,length(nVeh_s));
n_simulations = numel(e_fallback_strategies);
count = 0;

for iFallback = 1:length(fallback_types)
    for iVeh = 1:length(nVeh_s)
        options.fallback_type = fallback_types{iFallback};
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
            e_fallback_strategies{(iFallback-1)*random_times+iRandom,iVeh} = EvaluationParl(options);

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

average_random_simulations = @(data) reshape(mean(reshape(data,random_times,[]),1),length(fallback_types),[]);

% computation time of different parts
CT_group_vehs = cellfun(@(c) c.runtime_group_vehs, e_fallback_strategies);
% average of the random simulations
CT_group_vehs = average_random_simulations(CT_group_vehs);

CT_determine_couplings = cellfun(@(c) c.runtime_determine_couplings, e_fallback_strategies);
CT_determine_couplings = average_random_simulations(CT_determine_couplings);

CT_assign_priority = cellfun(@(c) c.runtime_assign_priority, e_fallback_strategies);
CT_assign_priority = average_random_simulations(CT_assign_priority);

CT_graph_search = cellfun(@(c) c.runtime_graph_search, e_fallback_strategies);
CT_graph_search = average_random_simulations(CT_graph_search);

% total runtime
t_total = cellfun(@(c) c.t_total, e_fallback_strategies);
t_total = average_random_simulations(t_total);

% fallback rate

fallback_rate = cellfun(@(c) c.fallback_rate*100, e_fallback_strategies);
fallback_rate = average_random_simulations(fallback_rate);

% average speed
speed_average = cellfun(@(c) c.average_speed, e_fallback_strategies);
speed_average = average_random_simulations(speed_average);

fig_x = 14;     fig_y = 6; % [cm]
x_margin = 0;   y_margin = 0; 
fig_x_position = fig_x - 2*x_margin;
fig_y_position = fig_y - 2*y_margin;

% options for plot 
plot_options_localFB = struct('LineWidth',.6,'Marker','*','MarkerSize',4);
plot_options_globalFB = struct('LineWidth',.6,'Marker','o','MarkerSize',4);
plot_options_noFB = struct('LineWidth',.6,'Marker','square','MarkerSize',4);

file_name = 'evalFallback';
fig = figure('Name',file_name);
set(fig, 'Units','centimeters', 'Position',[0 0 fig_x_position fig_y_position]/2)
set(fig, 'PaperUnits','centimeters','PaperSize',[fig_x fig_y],'PaperOrientation','portrait',...
    'PaperPosition', [x_margin -0.15 fig_x_position fig_y_position+0.1])

t_fig = tiledlayout(1,3,'Padding','tight','TileSpacing','tight');

% total run time
nexttile
plot(nVeh_s,t_total(1,:),plot_options_localFB);
hold on 
plot(nVeh_s,t_total(2,:),plot_options_globalFB);
plot(nVeh_s,t_total(3,:),plot_options_noFB);
grid on
xlabel({'$n_{veh}$','(a) Maximum runtime.'},'Interpreter','latex')
ylabel('$t_{max}\:[s]$','Interpreter','latex')
xlim([min(nVeh_s) max(nVeh_s)])
xticks(nVeh_s)

% average speed
nexttile
plot(nVeh_s,speed_average(1,:),plot_options_localFB);
hold on 
plot(nVeh_s,speed_average(2,:),plot_options_globalFB);
plot(nVeh_s,speed_average(3,:),plot_options_noFB);
grid on
l = legend({'Local fallback','Global fallback','Fallback not allowed'},'Orientation','horizontal','NumColumns',3);
l.Layout.Tile = 'north';
xlabel({'$n_{veh}$','(b) Average speed of all vehicles.'},'Interpreter','latex')
ylabel('$\overline{v}\:[m/s]$','Interpreter','latex')
xlim([min(nVeh_s) max(nVeh_s)])
xticks(nVeh_s)

% fallback rate
nexttile
plot(nVeh_s,fallback_rate(1,:),plot_options_localFB);
hold on 
plot(nVeh_s,fallback_rate(2,:),plot_options_globalFB);
plot(nVeh_s,fallback_rate(3,:),plot_options_noFB);
grid on
xlabel({'$n_{veh}$','(c) Average fallback rate.'},'Interpreter','latex')
ylabel('$\overline{p}_{FB}\:[\%]$','Interpreter','latex')
xlim([min(nVeh_s) max(nVeh_s)])
xticks(nVeh_s)

% xlabel(t_fig,'Compare different fallback strategies','FontSize',9,'FontName','Times New Roman')
% save fig
e_fallback_strategies{1}.save_fig(fig,file_name)

