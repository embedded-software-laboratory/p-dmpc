% TODO Grayscale friendly color map, use for both MATLAB plots and Latex
function results = spp_book(visu_options)
    % spp_book Generates evaluation results for the SPP book chapter
arguments
    visu_options.do_plot_online      (1,1) logical = 0;
    visu_options.is_video_exported   (1,1) logical = 0;
end

    %% Recursive feasibility
    opt = OptionsMain;
    opt.recursive_feasibility = 1;
    opt.is_load_mpa = 0;
    opt.Hp = 3;
    opt.trim_set = 1;
    veh = Vehicle();
    model = BicycleModel(veh.Lf, veh.Lr);
    sce_rec_1 = Scenario;
    sce_rec_1.mpa = MotionPrimitiveAutomaton(model,opt);
    sce_rec_1.options = opt;

    export_fig_config = ExportFigConfig.spp_book_small( ...
        "paperheight", 3 ...
    );
    plot_mpa_over_time( ...
        sce_rec_1, ...
        'do_export',true, ...
        'export_fig_cfg',export_fig_config ...
    );

    opt.recursive_feasibility = 0;
    sce_rec_0 = Scenario;
    sce_rec_0.mpa = MotionPrimitiveAutomaton(model,opt);
    sce_rec_0.options = opt;
    plot_mpa_over_time( ...
        sce_rec_0, ...
        'do_export',true, ...
        'export_fig_cfg',export_fig_config ...
    );


    %% Dynamic priorities
    % --------------------------------------------------------------------------
    close all
    disp('Evaluating dynamic priority assignment strategies.')
    

    % Commonroad
    priority_assignment_algorithms = {
        %'STAC_priority'
        %'right_of_way_priority'
        'FCA_priority'
        'random_priority'
        'constant_priority'
        'coloring_priority'
    }; 

    % scenarios as in Jianyes Eval
    options = OptionsMain;
    options.trim_set = 9;
    options.T_end = 180;
    options.Hp = 8;
    options.isPB = true;
    options.is_sim_lab = true;
    options.visu = [visu_options.do_plot_online, false];
    options.strategy_consider_veh_without_ROW = '2'; % '2': consider currently occupied area as static obstacle
    options.isAllowInheritROW = true;
    options.strategy_enter_lanelet_crossing_area = '1'; % 1: no constraint on entering the crossing area 
    options.collisionAvoidanceMode = 1;
    options.isSaveResult = 1;
    options.isSaveResultReduced = 1;
    options.scenario_name = 'Commonroad';

    % visualization for video
    options.optionsPlotOnline.isShowCoupling = true;
    

    nsVeh = 1:20;
    % number of different random scenarios per priority assignment and #vehicles
    seeds = 1:9;

    scenarios = commonroad_random(options, nsVeh, seeds);
    
    plot_mpa(scenarios(1,1), ...
        "y_lim",[-0.05, 0.85], ...
        "x_lim", [-37,37],...
        'export_fig_cfg',export_fig_config, ...
        "do_export",true ...
    );

    results = run_scenario_with_priority_algorithm( ...
        scenarios, priority_assignment_algorithms ...
    );
    
    % plot Computation levels histogram excluding deadlock
    plot_levels(results);
    % plot deadlock-free runtime
    plot_runtime(results);

    % Export videos
    export_desired_videos(results);
end