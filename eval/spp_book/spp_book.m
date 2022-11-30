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
    options.isParl = true;
    options.is_sim_lab = true;
    options.visu = [visu_options.do_plot_online, false];
    options.strategy_consider_veh_without_ROW = '2'; % '2': consider currently occupied area as static obstacle
    options.isAllowInheritROW = true;
    options.strategy_enter_lanelet_crossing_area = '1'; % 1: no constraint on entering the crossing area 
    options.collisionAvoidanceMode = 1;
    options.consider_RSS = false;
    options.isSaveResult = 1;
    options.isSaveResultReduced = 1;

    % visualization for video
    options.optionsPlotOnline.isShowCoupling = true;
    

    nsVeh = 1:20;
    % number of different random scenarios per priority assignment and #vehicles
    nSce = 9;

    scenarios = cell(length(nsVeh),nSce);
    results = cell(length(nsVeh),length(priority_assignment_algorithms),nSce);

    disp('Creating scenarios...')
    for inVeh = 1:length(nsVeh)
        for iSce = 1:nSce
            random_stream = RandStream('mt19937ar','Seed',iSce);
            options.amount = nsVeh(inVeh);
            % options.scenario_name = ['Clover-' num2str(options.amount) 'Vehicles']; % TODO get rid of dependencies on scenario name
            options.scenario_name = 'Commonroad';
            veh_ids = sort(randsample(random_stream,1:40,options.amount),'ascend');
            options.veh_ids = veh_ids;
            scenario = commonroad(options, options.veh_ids, 0, 0, options.is_sim_lab);
            scenario.random_stream = random_stream;
            scenario.name = options.scenario_name;
            scenario.manual_vehicle_id = 0;
            scenario.second_manual_vehicle_id = 0;
            scenario.vehicle_ids = options.veh_ids;
            scenario.mixedTrafficCollisionAvoidanceMode = options.collisionAvoidanceMode;
            for iVeh = 1:options.amount
                % initialize vehicle ids of all vehicles
                scenario.vehicles(iVeh).ID = scenario.vehicle_ids(iVeh);
            end
            scenarios{inVeh,iSce} = scenario;
        end
    end
    
    plot_mpa(scenarios{1,1}, ...
        "y_lim",[-0.05, 0.85], ...
        "x_lim", [-37,37],...
        'export_fig_cfg',export_fig_config, ...
        "do_export",true ...
    );

    n_simulations = length(nsVeh)*nSce*length(priority_assignment_algorithms);
    count = 0;
    disp('Starting simulations...')
    for inVeh = 1:length(nsVeh)
        disp(['# Vehicles: ',num2str(nsVeh(inVeh))])
        for i_priority = 1:length(priority_assignment_algorithms)
            disp(['Priority Assignment Algorithm: ',...
                priority_assignment_algorithms{i_priority}] ...
            )
            for iSce = 1:nSce
                scenarios{inVeh,iSce}.options.priority = ... 
                    priority_assignment_algorithms{i_priority};
                scenarios{inVeh,iSce}.controller_name = strcat(...
                    "seq. PB-RHGS ", ...
                    priority_assignment_algorithms{i_priority}...
                );
                % run simulation
                results_full_path = FileNameConstructor.get_results_full_path(...
                    scenarios{inVeh,iSce}.options...
                );
                if isfile(results_full_path)
                    disp('File already exists.')
                    r = load(results_full_path);
                    result = r.result;
                else
                    % run simulation
                    [result,~,~] = main(scenarios{inVeh,iSce});
                end
                results{inVeh,i_priority,iSce} = clean_result(result);

                % evaluate
                %e_differentNumVehs{i_priority} = EvaluationParl(results_full_path,[0,options.T_end]);
                
                % display progress
                count = count + 1;
                disp(['--------Progress ' num2str(count) '/' num2str(n_simulations) ': done--------'])
            end
        end
    end

    
    
    % plot Computation levels histogram excluding deadlock
    plot_levels(results);
    % plot deadlock-free runtime
    plot_runtime(results);

    % Export videos
    export_desired_videos(results);
end


function result = clean_result(result_in)
    result.scenario = result_in.scenario;
    result.is_deadlock = result_in.is_deadlock;
    result.priority = result_in.priority;
    result.t_total = result_in.t_total;
    result.nSteps = result_in.nSteps;
    result.controller_runtime = result_in.controller_runtime;
    result.output_path = result_in.output_path;
    result.iteration_structs = result_in.iteration_structs;
    result.directed_coupling = result_in.directed_coupling;
end