function free_flow_speed = get_free_flow_speed(options)

    arguments
        options (1, 1) Config;
    end

    %GET_FREE_FLOW_SPEED Free flow speed is the speed that vehicles could travel if they are not hindered by others
    % Inputs
    %   dt_seconds: sample time [s]
    % Outputs
    %   free_flow_speed: free flow speed [m/s]

    % prepare simulation options
    options.environment = Environment.Simulation;
    options.should_save_result = true;
    options.coupling = CouplingStrategies.no_coupling;

    if resultExists(options)
        disp('File already exists.')
    else
        % run simulation
        main(options);
    end

    % evaluate
    evaluation = EvaluationParl(options);

    free_flow_speed = evaluation.average_speed;
    disp(['Free-flow speed for sample time ' num2str(options.dt_seconds) ' s: ' num2str(free_flow_speed) ' m/s.'])

end
