%% Generate data: different allowed number of computation levels and different sample time
% prepare simulation options
options = OptionsMain;
options.scenario_name = 'Commonroad';
options.amount = 20;
options.is_prioritized = true;
options.dt = 0.2;
options.recursive_feasibility = true;

options.is_save_mpa = true;
options.is_load_mpa = true;

veh = Vehicle;
model = BicycleModel(veh.Lf, veh.Lr);

is_use_dynamic_programming = [true; false];
Hp_s = 1:12;
trim_set_s = [9, 13];

e_offline_RA = cell(length(trim_set_s), 1);

n_simulations = length(trim_set_s) * length(is_use_dynamic_programming) * length(Hp_s);
count = 0;

for i = 1:length(trim_set_s)
    % different trim set
    options.trim_set = trim_set_s(i);
    e_offline_RA{i} = inf(length(is_use_dynamic_programming), length(Hp_s));

    for j = 1:length(is_use_dynamic_programming)
        % whether use dynamic programming
        options.is_use_dynamic_programming = is_use_dynamic_programming(j);

        for hp_i = 1:length(Hp_s)
            % different prediction horizon
            options.Hp = Hp_s(hp_i);

            if options.Hp >= 9 && ~options.is_use_dynamic_programming
                % computation time will be too long for large preiction horizons
                % if dynamic programming is not used
                continue
            end

            % run
            mpa = MotionPrimitiveAutomaton(model, options);
            offline_reachability_computation_time = mpa.offline_reachability_computation_time;

            % store data
            e_offline_RA{i}(j, hp_i) = offline_reachability_computation_time;

            if options.is_use_dynamic_programming
                disp(['Run time with dynamic programming (Hp=' num2str(options.Hp) '): ' num2str(offline_reachability_computation_time)])
            else
                disp(['Run time without dynamic programming (Hp=' num2str(options.Hp) '): ' num2str(offline_reachability_computation_time)])
            end

            % display progress
            count = count + 1;
            disp(['--------Progress ' num2str(count) '/' num2str(n_simulations) ': done--------'])

        end

    end

end

disp('--------Finished--------')

%% Plot
set(0, 'DefaultTextFontname', 'Times New Roman');
set(0, 'DefaultAxesFontName', 'Times New Roman');
set(0, 'defaultTextFontSize', 11)
set(0, 'defaultAxesFontSize', 11)

clear plot_line_options
plot_line_options{1, 1} = struct('LineWidth', 0.6, 'Color', '#A2142F', 'LineStyle', '-', 'Marker', '*', 'MarkerSize', 4); % DP, complex trim primitives
plot_line_options{1, 2} = struct('LineWidth', 0.6, 'Color', '#A2142F', 'LineStyle', '--', 'Marker', '+', 'MarkerSize', 4); % brute-force, complex trim primitives
plot_line_options{2, 1} = struct('LineWidth', 0.6, 'Color', '#0072BD', 'LineStyle', '-', 'Marker', '^', 'MarkerSize', 4); % DP, simple trim primitives
plot_line_options{2, 2} = struct('LineWidth', 0.6, 'Color', '#0072BD', 'LineStyle', '--', 'Marker', 'v', 'MarkerSize', 4); % brute-force, simple trim primitives

fig_x = 11; fig_y = 8.2; % [cm]
x_margin = 0.05; y_margin = 0.05;
fig_x_position = fig_x - 2 * x_margin;
fig_y_position = fig_y - 2 * y_margin - 0.1;

file_name = 'evalOfflineRA';
fig = figure('Name', file_name);
set(fig, 'Units', 'centimeters', 'Position', [0 0 fig_x_position fig_y_position] / 2)
set(fig, 'PaperUnits', 'centimeters', 'PaperSize', [fig_x fig_y], 'PaperOrientation', 'portrait', ...
    'PaperPosition', [x_margin 0 fig_x_position fig_y_position + 0.25])

t_fig = tiledlayout(1, 1, 'Padding', 'tight', 'TileSpacing', 'none');

% plot computation time
nexttile
grid on
hold on
clear p
p(1) = plot(Hp_s, e_offline_RA{1}(1, :), plot_line_options{1, 1});
p(2) = plot(Hp_s, e_offline_RA{1}(2, :), plot_line_options{1, 2});
p(3) = plot(Hp_s, e_offline_RA{2}(1, :), plot_line_options{2, 1});
p(4) = plot(Hp_s, e_offline_RA{2}(2, :), plot_line_options{2, 2});
legend(p, {'DP (complex MPA)', 'Brute-force (complex MPA)', ...
              'DP (simple MPA)', 'Brute-force (simple MPA)'}, 'Location', 'best', 'Box', 'off');

set(gca, 'YScale', 'log')
xlabel({'$H_p$'}, 'Interpreter', 'latex');
ylabel('$t_c\:[s]$', 'Interpreter', 'latex')
xticks(Hp_s)
yticks([1e-4 1e-3 1e-2 1e-1 1 10 100 1000 10000])
xlim([Hp_s(1) Hp_s(end)])
ylim([1e-3 1e4])
% save fig
EvaluationParl.save_fig(fig, file_name)
