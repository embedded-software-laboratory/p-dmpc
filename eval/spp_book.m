% TODO Grayscale friendly color map, use for both MATLAB plots and Latex
function spp_book(options)
    % spp_book Generates evaluation results for the SPP book chapter
arguments
    options.do_plot_online      (1,1) logical = 1;
    options.is_video_exported   (1,1) logical = 0;
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
    s(1) = moving_obstacle_scenario(do_plot_online=options.do_plot_online);
    plot_mpa(s(1),"y_lim",[-0.05, 0.85]);
    results(1) = main(s(1));
    
    % SGS
    s(2) = moving_obstacle_scenario(...
        is_start_end=1,...
        do_plot_online=options.do_plot_online...
    );
    sub_controller = StartEndPlanner();
    s(2).sub_controller = @sub_controller.run;
    results(2) = main(s(2));

    % *Overview*
    nr = numel(results);

    filepath_text = fullfile('results', 'rhgs_vs_gs.txt');
    approaches = {'RHGS','SGS'};

    step_indices = [1 5 9 13];
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
    for r = results
        if options.is_video_exported
            exportVideo(r);
        end
    end



    %% Dynamic priorities
    % --------------------------------------------------------------------------
    disp('Evaluating dynamic priority assignment strategies.')
    % TODO check that all prio algorithms work
    priority_assignment_algorithms = {
        'right_of_way_priority'
        'FCA_priority'
        'topo_priority'
        'coloring'
        'random_priority'
        'constant_priority'
    };

    % TODO scenarios as in Jianyes Eval
    options = OptionsMain;
    options.scenario_name = 'Commonroad';
    options.trim_set = 9;
    options.T_end = 20;
    options.Hp = 5; % TODO 10
    options.isPB = true;

    scenarios = [];
    
    e_differentNumVehs = cell(length(priority_assign_options),1);
    n_simulations = numel(e_differentNumVehs);
    count = 0;

    % Commonroad
    % TODO OPT coupling based on lanelets + distance, simpler than Reachability Analysis
    results = [];
    for nVeh = 10:11 % TODO 10:20
        for priority = priority_assignment_algorithms
            for s = scenarios
                % create config
                s.options.priority = priority;
                s.options.amount = nVeh;

                % run simulation
                results_full_path = FileNameConstructor.get_results_full_path(options);
                if isfile(results_full_path)
                    disp('File already exists.')
                else
                    % run simulation
                    results(end+1) = main(s);
                end

                % evaluate
                e_differentNumVehs{i_priority} = EvaluationParl(results_full_path,[0,options.T_end]);
                
                % display progress
                count = count + 1;
                disp(['--------Progress ' num2str(count) '/' num2str(n_simulations) ': done--------'])
            end
        end
    end

    
    % Eval, zb.
    % plot Computation levels histogram
    % plot deadlock-free runtime