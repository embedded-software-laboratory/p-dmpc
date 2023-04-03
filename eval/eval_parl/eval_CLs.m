%% Generate data: different allowed number of computation levels and different sample time
% prepare simulation options
options = OptionsMain;
options.environment = Environment.Simulation;
options.result_name = '';
options.scenario_name = 'Commonroad';
options.trim_set = 9;
options.Hp = 6;
options.T_end = 8;
options.priority = 'STAC_priority';
options.is_prioritized = true;
options.allow_priority_inheritance = false;
options.strategy_consider_veh_without_ROW = '3';
options.strategy_enter_lanelet_crossing_area = '1';
options.should_save_result = true;
options.is_plot_online = false;
options.is_eval = false;
options.dt = 0.2;

options.veh_ids = [11, 18, 19, 20, 26, 27, 29, 31, 32, 40];
options.amount = numel(options.veh_ids);

max_num_CLs_all = [1, 2:2:8, options.amount];

e_CLs = cell(length(max_num_CLs_all), 1);
n_simulations = numel(e_CLs);
count = 0;

for i_CL = 1:length(max_num_CLs_all)
    options.max_num_CLs = max_num_CLs_all(i_CL);

    full_path = FileNameConstructor.get_results_full_path(options);

    if isfile(full_path)
        disp('File already exists.')
    else
        % run simulation
        if exist('options', 'var') && exist('scenario', 'var')
            [~, ~, ~] = main(options, scenario);
        else
            [~, scenario, ~] = main(options);
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
if exist('Scenario', 'var')
    free_flow_speed = FreeFlowSpeed(scenario);
else
    free_flow_speed = FreeFlowSpeed();
end

disp('--------Finished--------')

%% Plot
set(0, 'DefaultTextFontname', 'Times New Roman');
set(0, 'DefaultAxesFontName', 'Times New Roman');
set(0, 'defaultTextFontSize', 11)
set(0, 'defaultAxesFontSize', 11)

CT_graph_search = cellfun(@(c) c.runtime_graph_search, e_CLs);
CT_graph_search_average = cellfun(@(c) c.runtime_graph_search_average, e_CLs);
CT_graph_search_pure_sequential = cellfun(@(c) c.runtime_graph_search_pure_sequential, e_CLs);
CT_graph_search_pure_sequential_average = cellfun(@(c) c.runtime_graph_search_pure_sequential_average, e_CLs);
CT_graph_search(end + 1) = CT_graph_search_pure_sequential(end);
CT_graph_search_average(end + 1) = CT_graph_search_pure_sequential_average(end);

CT_total_max = cellfun(@(c) c.runtime_max, e_CLs);
% CT_total_max = average_random_simulations(CT_total_max);

% number of coupling
num_couplings = cellfun(@(c) c.num_couplings, e_CLs);
num_couplings(end + 1) = num_couplings(end);
num_couplings_ignored = cellfun(@(c) c.num_couplings_ignored, e_CLs);
num_couplings_ignored(end + 1) = num_couplings_ignored(end);
num_couplings_between_grps = cellfun(@(c) c.num_couplings_between_grps, e_CLs);
num_couplings_between_grps(end + 1) = num_couplings_between_grps(end);
num_couplings_between_grps_ignored = cellfun(@(c) c.num_couplings_between_grps_ignored, e_CLs);
num_couplings_between_grps_ignored(end + 1) = num_couplings_between_grps_ignored(end);

% maximum number of actual computation levels
CLs_num_max = cellfun(@(c) c.CLs_num_max, e_CLs);
CLs_num_max(end + 1) = max_num_CLs_all(end);

% average speed
speed_average = cellfun(@(c) c.average_speed, e_CLs);
speed_average(end + 1) = speed_average(end);

plot_line_options = {};
plot_line_options{1}{1} = struct('LineWidth', 0.6, 'Color', '#A2142F', 'LineStyle', '-', 'Marker', '*', 'MarkerSize', 4); % average
plot_line_options{1}{2} = struct('LineWidth', 0.6, 'Color', '#A2142F', 'LineStyle', '-.', 'Marker', '*', 'MarkerSize', 4); % maximum
plot_line_options{2}{1} = struct('LineWidth', 0.6, 'Color', '#7E2F8E', 'LineStyle', '-', 'Marker', '^', 'MarkerSize', 4);
plot_line_options{2}{2} = struct('LineWidth', 0.6, 'Color', '#7E2F8E', 'LineStyle', '-.', 'Marker', '^', 'MarkerSize', 4);
plot_line_options{3}{1} = struct('LineWidth', 0.6, 'Color', '#0072BD', 'LineStyle', '-', 'Marker', 'o', 'MarkerSize', 4);
plot_line_options{3}{2} = struct('LineWidth', 0.6, 'Color', '#0072BD', 'LineStyle', '-.', 'Marker', 'o', 'MarkerSize', 4);

fig_x = 14; fig_y = 10; % [cm]
x_margin = 0; y_margin = 0;
fig_x_position = fig_x - 2 * x_margin;
fig_y_position = fig_y - 2 * y_margin;

file_name = 'evalCLs_presentation';
fig = figure('Name', file_name);
set(fig, 'Units', 'centimeters', 'Position', [0 0 fig_x_position fig_y_position] / 2)
set(fig, 'PaperUnits', 'centimeters', 'PaperSize', [fig_x fig_y], 'PaperOrientation', 'portrait', ...
    'PaperPosition', [0 0 fig_x_position - 0.1 fig_y_position])

t_fig = tiledlayout(2, 2, 'Padding', 'tight', 'TileSpacing', 'tight');

X_string = string(max_num_CLs_all);
X_string(1) = 'Parl.';
X_string(end) = 'Alr.';
X_string(end + 1) = 'Seq.';
X_cat = categorical(X_string);
X_cat = reordercats(X_cat, X_string);

% average speed
nexttile
grid on
hold on
plot(X_cat, speed_average, plot_line_options{1}{1});
free_flow_speed_i = free_flow_speed.free_flow_speed(free_flow_speed.sample_time == options.dt);
FFS = yline(free_flow_speed_i, '--b', 'LineWidth', 0.6);
legend(FFS, {'Free-flow'}, 'FontSize', 9, 'Location', 'best', 'Interpreter', 'latex')

xlabel({'(a) Average speed.'}, 'Interpreter', 'latex')
ylabel('$\overline{v}\:[m/s]$', 'Interpreter', 'latex')
ylim([0.6, .75])
set(gca, 'XTickLabel', '')

% average number of couplings
nexttile
grid on
hold on
clear c
c(1) = plot(X_cat, num_couplings, plot_line_options{1}{1});
c(2) = plot(X_cat, num_couplings_between_grps, plot_line_options{2}{1});
legend(c, {'Total', 'Cross-group'}, 'FontSize', 9, 'Location', 'best');
ylim([0 10])
xlabel({'(b) Number of couplings.'}, 'Interpreter', 'latex')
ylabel('$\overline{n}_{c}$', 'Interpreter', 'latex')
set(gca, 'XTickLabel', '')

% maximum allowed and actual number of computation levels
nexttile
grid on
hold on
p_CLs(1) = plot(X_cat, [max_num_CLs_all, max_num_CLs_all(end)], plot_line_options{1}{1});
p_CLs(2) = plot(X_cat, CLs_num_max, plot_line_options{2}{1});
legend(p_CLs, {'Allowed', 'Actual'}, 'FontSize', 9, 'Location', 'northwest')
xlabel({'$n_{CLs}$', '(c) Computation levels.'}, 'Interpreter', 'latex');
ylabel('$n_{CLs},n_{CLs}^{act.}$', 'Interpreter', 'latex')
xtickangle(0)

% plot computation time
nexttile
grid on
hold on
clear p
p(1) = plot(X_cat, CT_graph_search, plot_line_options{3}{2});
p(2) = plot(X_cat, CT_graph_search_average, plot_line_options{3}{1});
ylim([0, 0.2])
legend(p, {'Maximum', 'Average'}, 'Location', 'best');

xlabel({'$n_{CLs}$', '(d) Trajectory planning time.'}, 'Interpreter', 'latex');
ylabel('$t_c\:[s]$', 'Interpreter', 'latex')
xtickangle(0)
% save fig
e_CLs{1}.save_fig(fig, file_name)
%% export video
options.max_num_CLs = 1;
full_path = FileNameConstructor.get_results_full_path(options);
load(full_path, 'result')

result.scenario.options.options_plot_online.is_video_mode = true;
result.scenario.options.options_plot_online.plot_coupling = false;
result.scenario.options.options_plot_online.plot_priority = false;

videoExportSetup.framerate = 5;
exportVideo(result, videoExportSetup)
