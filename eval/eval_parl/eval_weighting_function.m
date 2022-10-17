%% Evaluate the weighting function for couplings

% prepare simulation options
options = OptionsMain;
options.consider_RSS = false;
options.is_sim_lab = true;
options.customResultName = '';
options.scenario_name = 'Commonroad';
options.trim_set = 9;
options.Hp = 5;

options.T_end = 20;
options.dt = 0.2;
options.max_num_CLs = 2;
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
options.is_force_parallel_vehs_in_same_grp = false;

coupling_weight_mode = {'STAC','random','constant','optimal'};
% coupling_weight_mode = {'optimal'};
% Random choose different vehicles three times
count = 0;
e_weighting_function = cell(length(coupling_weight_mode),1);
n_simulations = numel(e_weighting_function);

options.amount = 18;

random_stream = RandStream('mt19937ar');
options.veh_ids = sort(randsample(random_stream,9:40,options.amount),'ascend');

for i = 1:length(coupling_weight_mode)
    options.coupling_weight_mode  = coupling_weight_mode{i};
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
    % display progress
    count = count + 1;
    e_weighting_function{i} = EvaluationParl(options);
    disp(['--------Progress ' num2str(count) '/' num2str(n_simulations) ': done--------'])
end

disp('--------Finished--------')

%% Plot
set(0,'DefaultTextFontname', 'Times New Roman');
set(0,'DefaultAxesFontName', 'Times New Roman');
set(0,'defaultTextFontSize',11)
set(0,'defaultAxesFontSize',11)

speed_average = cellfun(@(c) c.average_speed, e_weighting_function);

runtime_determine_couplings = cellfun(@(c) c.runtime_determine_couplings, e_weighting_function);
runtime_determine_couplings_average = cellfun(@(c) c.runtime_determine_couplings_average, e_weighting_function);

fallback_rate_s = cellfun(@(c) c.fallback_rate*100, e_weighting_function);

fig_x = 14.5;     fig_y = 6; % [cm]
x_margin = 0;   y_margin = 0; 
fig_x_position = fig_x - x_margin;
fig_y_position = fig_y - y_margin;

file_name = 'evalWeightingFunction_sameNumVehs';
fig = figure('Name',file_name);
set(fig, 'Units','centimeters', 'Position',[0 0 fig_x_position fig_y_position]/2)
set(fig, 'PaperUnits','centimeters','PaperSize',[fig_x fig_y],'PaperOrientation','portrait',...
    'PaperPosition', [-0.15 -0.11 fig_x_position+0.55 fig_y_position+0.3])

t_fig = tiledlayout(1,2,'Padding','compact','TileSpacing','compact');
X_string = {'STAC','Random','Constant','Optimal'};
X_cat = categorical(X_string);
X_cat = reordercats(X_cat,X_string);

nexttile
% fallback rate
b3 = bar(X_cat,fallback_rate_s);
grid on
xtips3 = b3(1).XEndPoints;
ytips3 = b3(1).YEndPoints;
labels3 = string(round(b3(1).YData,3));
text(xtips3,ytips3,labels3,'HorizontalAlignment','center','VerticalAlignment','bottom')
ylim([0 6])
ylabel('$\overline{p}_{FB}\:[\%]$','Interpreter','latex')
xtickangle(0)
xlabel('(b) Average fallback rate.')

nexttile
% coupling determination time
b2 = bar(X_cat,runtime_determine_couplings_average);
grid on
xtips2 = b2(1).XEndPoints;
ytips2 = b2(1).YEndPoints;
labels2 = string(round(b2(1).YData,3));
text(xtips2-0.05,ytips2,labels2,'HorizontalAlignment','center','VerticalAlignment','bottom')
ylim([0 0.4])
ylabel('$t\:[s]$','Interpreter','latex')
xtickangle(0)
xlabel('(c) Average coupling determination time.')
yticks(0:0.1:0.4)

% save fig
e_weighting_function{1}.save_fig(fig,file_name)
