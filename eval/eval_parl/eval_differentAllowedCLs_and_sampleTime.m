%% Generate data: different allowed number of computation levels and different sample time
dt_s = 0.2:0.1:0.4;
max_num_CLs_all = 1:10;
% load('optionsMain.mat','options')
for i_dt = 1:length(dt_s)
    for i_CL = 1:length(max_num_CLs_all)
        
        % prepare simulation options
        options = OptionsMain;
        options.consider_RSS = false;
        options.is_sim_lab = true;
        options.customResultName = '';
        options.scenario = 'Commonroad';
        options.trim_set = 9;
        options.Hp = 5;
        options.dt = dt_s(i_dt);
        options.amount = 20;
        options.T_end = 20;
        options.priority = 'right_of_way_priority';
        options.isPB = true;
        options.isParl = true;
        options.isAllowInheritROW = true;
        options.max_num_CLs = max_num_CLs_all(i_CL);
        options.strategy_consider_veh_without_ROW = '3';
        options.strategy_enter_lanelet_crossing_area = '4';
        options.isSaveResult = true;
        options.visu = [false,false];
        options.is_eval = false;
        options.visualize_reachable_set = false;

        % run simulation
        if exist('options','var') && exist('scenario','var')
            [~,scenario,options] = main(options,scenario);
        else
            [~,scenario,options] = main(options);
        end

        disp([num2str(i_CL) ': done.'])
        disp('Pausing...')
        pause(3) % pause to cool the machine
    end
end
disp('Finished.')

%% Data preprocessing 
max_num_CLs_all = 1:10;
dt_s = 0.2:0.1:0.4;
evaluations = cell(length(dt_s),length(max_num_CLs_all));
for i_dt = 1:length(dt_s)
    disp(dt)
    for i_CL = 1:length(max_num_CLs_all)
        customResultName = '';
        scenario_name = 'Commonroad';
        controller_name = 'RHC-Parl';
        trim_set = 9;
        Hp = 5;
        dt = dt_s(i_dt);
        nVeh = 20;
        T_end = 20;
        priority_option = 'right_of_way_priority';
        isParl = true;
        isAllowInheritROW = true;
        max_num_CLs = max_num_CLs_all(i_CL);
        strategy_consider_veh_without_ROW = '3';
        strategy_enter_lanelet_crossing_area = '4';
        results_full_path = FileNameConstructor.get_results_full_path(customResultName,scenario_name,controller_name,trim_set,...
                        Hp,dt,nVeh,T_end,priority_option,isParl,isAllowInheritROW,max_num_CLs,strategy_consider_veh_without_ROW,strategy_enter_lanelet_crossing_area);
        evaluations{i_dt,i_CL} = EvaluationCommon(results_full_path);
        disp(i_CL)
    %     disp(['Average run time: ' num2str(evaluations{i_CL}.runtime_average) ' seconds.'])
    %     disp(['Top 8 maximum runtime: ' mat2str(round(evaluations{i_CL}.runtime_max,3)) ' seconds.'])
    end
end
%% Box plot
set(0,'DefaultTextFontname', 'Times New Roman');
set(0,'DefaultAxesFontName', 'Times New Roman');
set(0,'defaultTextFontSize',9)
set(0,'defaultAxesFontSize',9)
for i_dt = 1:length(dt_s)
    dt = dt_s(i_dt);
    runtime_s = cellfun(@(c) c.runtime_total_per_step, evaluations(i_dt,:), 'UniformOutput',false);
    speed_s = cellfun(@(c) c.average_speed_each_veh, evaluations(i_dt,:), 'UniformOutput',false);
    fallback_rate = cellfun(@(c) c.fallback_rate*100, evaluations(i_dt,:));
    is_save_fig = true;
    box_plot_different_CLs(dt,runtime_s,speed_s,fallback_rate,is_save_fig)
end

%% local function: box plot, compare different allowered number of computation levels
function box_plot_different_CLs(dt,runtime_s,speed_s,fallback_rate,is_save_fig)
    if nargin<=4
        is_save_fig = false;
    end
    max_num_CLs_all = 1:length(runtime_s);
    grp_runtime = cell2mat(arrayfun(@(i){i*ones(numel(runtime_s{i}),1)},(1:numel(runtime_s))')); 
    grp_speed = cell2mat(arrayfun(@(i){i*ones(numel(speed_s{i}),1)},(1:numel(speed_s))')); 

    fig = figure('Name','differentAllowedCLs');
    fig.Units = 'centimeters';
    fig.Position = [0 0 16 8];
    t_fig = tiledlayout(1,3,'TileSpacing','compact');
    % boxplot runtime
    % title(t_fig,'Compare different maximum allowed number of computation levels')
    % subplot(3,1,1)
    nexttile
    grid on
    hold on 
    boxplot(vertcat(runtime_s{:}),grp_runtime,'Colors','k','OutlierSize',4)
    % plot line indicating the sample time 
    p_dt = plot([0,length(runtime_s)+1],[dt,dt],'--b','LineWidth',1.0);
    ylabel('Runtime per step $[s]$','Interpreter','latex')
    ylim([0,0.42])
    % boxplot speed
    % subplot(3,1,2)
    nexttile
    grid on
    hold on
    boxplot(vertcat(speed_s{:}),grp_speed,'Colors','k','OutlierSize',4)
    ylabel('Average speed of each vehicle $[m/s]$','Interpreter','latex')
    ylim([0.3,0.75])
    % plot fallback rate
    % subplot(3,1,3)
    nexttile
    grid on
    hold on
    plot(max_num_CLs_all,fallback_rate,'-*k','LineWidth',0.5,'MarkerSize',4)
    xlim([max_num_CLs_all(1)-1,max_num_CLs_all(end)+0.5])
    ylim([0,3])
    ylabel('Fallback rate $[\%]$','Interpreter','latex')
    legend([p_dt],{'Sample time $[s]$'},'Location','southeast','Interpreter','latex')
    xticks(max_num_CLs_all)
    
    xlabel(t_fig,'Allowed number of computation levels','FontSize',9,'FontName','Times New Roman')
    title(t_fig,['Sample time: ' num2str(dt) ' seconds'],'FontSize',12,'FontName','Times New Roman')
    
    if is_save_fig
        % save fig to pdf
        set(fig,'Units','centimeters');
        pos = get(fig,'Position');
        set(fig,'PaperPositionMode','Auto','PaperUnits','centimeters','PaperSize',[pos(3), pos(4)])
        
        folder_target = fullfile(pwd,'fig');
        if ~isfolder(folder_target)
            % create target folder if not exist
            mkdir(folder_target)
        end
        
        file_name = ['compareDifferentMaximumAllowedNumberOfCLs_dt' num2str(dt)];
        file_name_svg = [file_name '.svg'];
        file_name_pdf = [file_name '.pdf'];
        full_path_svg = fullfile(folder_target,file_name_svg);
        full_path_pdf = fullfile(folder_target,file_name_pdf);

        print(fig,full_path_pdf,'-dpdf','-r0');
        print(fig,full_path_svg,'-dsvg','-r0');
    end
end