% Start from newest branch -- jianye eval
% TODO Grayscale friendly color map, use for both MATLAB plots and Latex
function spp_book(plot_online,is_video_exported)
    % spp_book Generates evaluation results for the SPP book chapter

    % TODO arguments block
    if nargin==0
        plot_online = 0;
        is_video_exported = 1;
    end

    % Settings:
    % t_horizon = 4 s
    % T = 0.2 s
    % -> N = 20 (TODO Check if possible computation time wise)
    % MPA: Jianyes MPA
    % adjust set_figure_properties; arguments block, so other settings possible

    %% Create MPA for experiments
    % TODO create MPA
    % TODO plot MPA (function in feature/large-mpa)

    %% Trajectory planning
    % --------------------------------------------------------------------------
    disp('Evaluating with one vehicle and moving obstacles.')


    s(1) = moving_obstacle_scenario(do_plot_online=0);
    results(1) = main(s(1));
%     overviewPlot(r,[17,21,25,28]); % TODO adjust
%     if is_video_exported
%         exportVideo(r);
%     end
    % TODO comparison to SGS. reuse code from fix/rhgs-eval
    s(2) = moving_obstacle_scenario(is_start_end=1,do_plot_online=1);
    sub_controller = StartEndPlanner();
    s(2).sub_controller = @sub_controller.run;
    results(2) = main(s(2));
    
    nr = numel(results);
    
    filepath_text = fullfile('results', 'rhgs_vs_gs.txt');
    approaches = {'RHGS','SGS'};
    
    % *Overview*
    fig = overviewPlot(results(1),[11, 15, 19, 22]);
    overviewPlot(results(2),[11, 15, 19, 22],fig,1);
    
    % *Computation time*
    runtimes = reshape([results.controller_runtime],[],nr);
    t_max = max(runtimes);
    t_median = median(runtimes);
    t = [t_max; t_median];
    
    % plot
    fig = figure;
    % X = categorical({'t_{max}','t_{median}'});
    % X = reordercats(X,{'t_{max}','t_{median}'});
    b = barh(1:2,t);
    set(gca, 'XScale','log');
    for i = 1:nr
        b(i).FaceColor = vehColor(i);
    end
    yticklabels({'t_{max}','t_{median}'})
    legend(approaches{:},'Location','best')
    xlabel('Computation Time [s]');
    
    set(gca,'Ydir','reverse');
    % ylim('tight')
    
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
        if is_video_exported
            exportVideo(r);
        end
    end



    %% Dynamic priorities
    % --------------------------------------------------------------------------
    disp('Evaluating dynamic priority assignment strategies.')
    priority_assignment_algorithms = ...
        {@paa_coloring, @right_of_way_priority, @paa_fca, @paa_random, @paa_constant};
    % TODO scenarios as in Jianyes Eval
    % TODO check that all prio algorithms work
    % TODO OPT coupling based on lanelets + distance, simpler than Reachability Analysis
    scenarios = 0;

    for nVeh = 10:20
        for paa = priority_assignment_algorithms
            for s = scenarios
                % create config
                % run simulation
                % Eval, zb.
                % plot Computation levels histogram
                % plot deadlock-free runtime
            end
        end
    end