%% Generate data: local/global/no fallback
fallback_types = {'localFallback','globalFallback','noFallback'};
nVeh_s = 26:35;

e_fallback_strategies = cell(length(fallback_types),length(nVeh_s));

% prepare simulation options
options = OptionsMain;
options.consider_RSS = false;
options.is_sim_lab = true;
options.customResultName = '';
options.scenario = 'Commonroad';
options.trim_set = 9;
options.Hp = 5;

options.T_end = 99999;
options.dt = 0.2;
options.max_num_CLs = 3;
options.priority = 'right_of_way_priority';
options.isPB = true;
options.isParl = true;
options.isAllowInheritROW = true;
options.isSaveResult = true;
options.isSaveResultReduced = false;
options.visu = [false,false];
options.is_eval = false;
options.visualize_reachable_set = false;
options.strategy_consider_veh_without_ROW = '3';
options.strategy_enter_lanelet_crossing_area = '4';

for iFallback = 1:length(fallback_types)
    for iVeh = 1:length(nVeh_s)
        options.fallback_type = fallback_types{iFallback};
        options.amount = nVeh_s(iVeh);
    
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
        e_fallback_strategies{iFallback,iVeh} = EvaluationParl(options);
    
        disp([num2str(iVeh) ': done.'])
    end
end
disp('--------Finished--------')

%% Plot
set(0,'DefaultTextFontname', 'Times New Roman');
set(0,'DefaultAxesFontName', 'Times New Roman');
set(0,'defaultTextFontSize',9)
set(0,'defaultAxesFontSize',9)

t_total_s = cellfun(@(c) c.t_total, e_fallback_strategies);
fallback_rate_s = cellfun(@(c) c.fallback_rate*100, e_fallback_strategies);
speed_average_s = cellfun(@(c) c.average_speed, e_fallback_strategies);

fig_x = 13;     fig_y = 6; % [cm]
x_margin = 0;   y_margin = 0; 
fig_x_position = fig_x - 2*x_margin;
fig_y_position = fig_y - 2*y_margin;

% options for plot 
plot_options_localFB = struct('LineWidth',.5,'Marker','*','MarkerSize',4);
plot_options_globalFB = struct('LineWidth',.5,'Marker','o','MarkerSize',4);
plot_options_noFB = struct('LineWidth',.5,'Marker','square','MarkerSize',4);

fig = figure('Name','differentFallbackStrategies');
set(fig, 'Units','centimeters', 'Position',[0 0 fig_x_position fig_y_position]/2)
set(fig, 'PaperUnits','centimeters','PaperSize',[fig_x fig_y],'PaperOrientation','portrait',...
    'PaperPosition', [x_margin y_margin fig_x_position fig_y_position])

legend_ = {'local fallback','global fallback','no fallback'};
t_fig = tiledlayout(1,3,'Padding','tight','TileSpacing','tight');


% total run time
nexttile
plot(nVeh_s,t_total_s(1,:),plot_options_localFB);
hold on 
plot(nVeh_s,t_total_s(2,:),plot_options_globalFB);
plot(nVeh_s,t_total_s(3,:),plot_options_noFB);
grid on
xlabel({'$n_{veh}$','(a). Maximum runtime.'},'Interpreter','latex')
ylabel('$t_{max}\:[s]$','Interpreter','latex')
xlim([min(nVeh_s) max(nVeh_s)])
% xticks(nVeh_s)

% average speed
nexttile
plot(nVeh_s,speed_average_s(1,:),plot_options_localFB);
hold on 
plot(nVeh_s,speed_average_s(2,:),plot_options_globalFB);
plot(nVeh_s,speed_average_s(3,:),plot_options_noFB);
grid on
legend(legend_,'Orientation','horizontal','Location','northoutside')
xlabel({'$n_{veh}$','(b). Average speed of all vehicles.'},'Interpreter','latex')
ylabel('$\overline{v}\:[m/s]$','Interpreter','latex')
xlim([min(nVeh_s) max(nVeh_s)])

% fallback rate
nexttile
plot(nVeh_s,fallback_rate_s(1,:),plot_options_localFB);
hold on 
plot(nVeh_s,fallback_rate_s(2,:),plot_options_globalFB);
plot(nVeh_s,fallback_rate_s(3,:),plot_options_noFB);
grid on
xlabel({'$n_{veh}$','(c). Fallback rate.'},'Interpreter','latex')
ylabel('$p_{FR}\:[\%]$','Interpreter','latex')
xlim([min(nVeh_s) max(nVeh_s)])

% xlabel(t_fig,'Compare different fallback strategies','FontSize',9,'FontName','Times New Roman')
% save fig
file_name = 'differentFallbackStrategies';
e_fallback_strategies{1}.save_fig(fig,file_name)


