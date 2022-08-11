%% Evaluate vehicle prioritizing algorithm
priority_assign_options = {'right_of_way_priority','random_priority','constant_priority'};
for i_priority = 1:length(priority_assign_options)
    % prepare simulation options
    options = OptionsMain;
    options.consider_RSS = false;
    options.is_sim_lab = true;
    options.customResultName = '';
    options.scenario = 'Commonroad';
    options.trim_set = 9;
    options.Hp = 5;
    options.dt = 0.2;
    options.T_end = 20;
    options.priority = priority_assign_options{i_priority};
    switch options.priority
        case 'right_of_way_priority'
            options.amount = 20;
        case 'random_priority'
            options.amount = 17;
        case 'constant_priority'
            options.amount = 14;
    end
    options.isPB = true;
    options.isParl = true;
    options.isAllowInheritROW = true;
    options.max_num_CLs = 3;
    options.strategy_consider_veh_without_ROW = '3';
    options.strategy_enter_lanelet_crossing_area = '4';
    options.isSaveResult = true;
    options.visu = [false,false];
    options.is_eval = false;
    options.visualize_reachable_set = false;

    % run simulation
    if exist('options','var') && exist('scenario','var')
        [~,~,~] = main(options,scenario);
    else
        [~,scenario,~] = main(options);
    end

    disp([num2str(i_priority) ': done.'])
    disp('Pausing...')
    pause(3) % pause to cool the machine
end
disp('Finished.')


%% Data preprocessing
