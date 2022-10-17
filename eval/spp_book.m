% TODO Grayscale friendly color map, use for both MATLAB plots and Latex
function results = spp_book(visu_options)
    % spp_book Generates evaluation results for the SPP book chapter
arguments
    visu_options.do_plot_online      (1,1) logical = 1;
    visu_options.is_video_exported   (1,1) logical = 0;
end

    % Settings:
    % t_horizon = 4 s
    % T = 0.2 s
    % -> N = 20 (TODO Check if possible computation time wise)
    % adjust set_figure_properties; arguments block, so other settings possible

    %% Trajectory planning
    % --------------------------------------------------------------------------
    disp('Evaluating with one vehicle and moving obstacles.')

    % RHGS
    s(1) = moving_obstacle_scenario(do_plot_online=visu_options.do_plot_online);
    plot_mpa(s(1),"y_lim",[-0.05, 0.85]);
    
    % SGS
    s(2) = moving_obstacle_scenario(...
        is_start_end=1,...
        do_plot_online=visu_options.do_plot_online...
    );
    sub_controller = StartEndPlanner();
    s(2).sub_controller = @sub_controller.run;

    % run simulation
    for i = 1:length(s)
        results_full_path = FileNameConstructor.get_results_full_path(s(i).options);
        if isfile(results_full_path)
            disp('File already exists.')
            r_loaded = load(results_full_path);
            results(i) = r_loaded.result; %#ok<AGROW>
        else
            % run simulation
            results(i) = main(s(i)); %#ok<AGROW> 
        end
    end

    % *Overview*
    nr = numel(results);

    filepath_text = fullfile('results', 'rhgs_vs_gs.txt');
    approaches = {'RHGS','SGS'};

    step_indices = [1 9];
    fig = overviewPlot(results(1),step_indices);
    overviewPlot(results(2),step_indices,fig,1);

    % *Computation time*
    % prepare
    runtimes = reshape([results.controller_runtime],[],nr);
    t_max = max(runtimes);
    t_median = median(runtimes);
    t = [t_max; t_median];

    % plot
    fig = figure;
    b = barh(1:2,t);
    set(gca, 'XScale','log');
    for i = 1:nr
        b(i).FaceColor = vehColor(i);
    end
    yticklabels({'t_{max}','t_{median}'})
    legend(approaches{:},'Location','best')
    xlabel('Computation Time [s]');

    set(gca,'Ydir','reverse');

    set_figure_properties(fig, 'paper',3);
    xmin = min(t,[],'all');
    xmax = max(t,[],'all');
    xlim([0.5*xmin, 1.5*xmax])
    filepath = fullfile('results', 'rhgs_vs_gs_computation_time.pdf');
    export_fig(fig, filepath);
    close(fig);

    fileID = fopen(filepath_text,'w');
    for iApp = 1:numel(approaches)
        fprintf(fileID ...
            ,'%4s: t_max %7.2f ms (%5.1f times faster), t_median %7.2f ms (%3.1f times faster)\n' ...
            , approaches{iApp} ...
            , t_max(iApp)*1000 ...
            , t_max(2)/t_max(iApp) ...
            , t_median(iApp)*1000 ...
            , t_median(2)/t_median(iApp));
    end
    fclose(fileID);


    % *Objective value*
    pos_ref = reshape( ...
        results(2).iteration_structs{1}.referenceTrajectoryPoints, ...
        size(results(2).iteration_structs{1}.referenceTrajectoryPoints,2), ...
        size(results(2).iteration_structs{1}.referenceTrajectoryPoints,3) ...
    )';
    % First reference point is meant for second actual point
    pos_ref = pos_ref(:,1:end-1);

    objective_value = zeros(1,nr);
    for ir = 1:nr
        iter_struct_array = [results(ir).iteration_structs{:}];
        state = reshape([iter_struct_array.x0],numel(iter_struct_array(1).x0),[]);
        % First state point has no reference point
        pos_act = state(1:2,2:end);
        objective_value(ir) = sum(vecnorm(pos_ref-pos_act).^2);
    end

    % plot
    fig = figure;
    b = barh(1,objective_value);
    for i = 1:nr
        b(i).FaceColor = vehColor(i);
    end
    legend('RHGS','SGS','Location','best')
    xlabel('Objective Function Value');
    yticks('');

    ylim([0.5 2])

    set(gca,'Ydir','reverse');

    set_figure_properties(fig, 'paper',2.5);
    filepath = fullfile('results', 'rhgs_vs_gs_objective_value.pdf');
    export_fig(fig, filepath);
    close(fig);

    fileID = fopen(filepath_text,'a');
    for iApp = 1:numel(approaches)
        fprintf(fileID ...
            ,'%4s: J %7.2f (%5.1f times worse)\n' ...
            , approaches{iApp} ...
            , objective_value(iApp) ...
            , objective_value(iApp)/objective_value(2) );
    end
    fclose(fileID);



    % *Video*
    if visu_options.is_video_exported
        for r = results
            exportVideo(r);
        end
    end

    %% Dynamic priorities
    % --------------------------------------------------------------------------
    disp('old')
    close all
    disp('Evaluating dynamic priority assignment strategies.')
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
    options.T_end = 120; % TODO is this long enough?
    options.Hp = 10;
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
    

    % TODO input list of different numbers of vehicles
    nsVeh = 1:20;% TODO 10:20
    % TODO input scalar number of different random scenarios per priority assignment and #vehicles
    nSce = 5;

    scenarios = cell(length(nsVeh),nSce);
    results = cell(length(nsVeh),length(priority_assignment_algorithms),nSce);

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

    n_simulations = length(nsVeh)*nSce*length(priority_assignment_algorithms);
    count = 0;

    % Commonroad
    for inVeh = 1:length(nsVeh)
        disp(['# Vehicles: ',num2str(nsVeh(inVeh))])
        for i_priority = 1:length(priority_assignment_algorithms)
            disp(['Priority Assignment Algorithm: ', priority_assignment_algorithms{i_priority}])
            for iSce = 1:nSce
                scenarios{inVeh,iSce}.options.priority = priority_assignment_algorithms{i_priority};
                scenarios{inVeh,iSce}.controller_name = strcat(...
                    "seq. PB-RHGS ", ...
                    priority_assignment_algorithms{i_priority}...
                );
                % run simulation
                results_full_path = FileNameConstructor.get_results_full_path(scenarios{inVeh,iSce}.options);
                if isfile(results_full_path)
                    disp('File already exists.')
                    r_loaded = load(results_full_path);
                    results{inVeh,i_priority,iSce} = r_loaded.result;
                else
                    % run simulation
                    [results{inVeh,i_priority,iSce},~,~] = main(scenarios{inVeh,iSce});
                end

                % evaluate
                %e_differentNumVehs{i_priority} = EvaluationParl(results_full_path,[0,options.T_end]);
                
                % display progress
                count = count + 1;
                disp(['--------Progress ' num2str(count) '/' num2str(n_simulations) ': done--------'])
            end
        end
    end

    
    
    % TODO Eval, zb.
    % plot Computation levels histogram excluding deadlock
    eval_plot_levels(results{end,:,:});
    % plot deadlock-free runtime
    eval_plot_runtime(results);

    % Export videos
    export_desired_videos(results);
end