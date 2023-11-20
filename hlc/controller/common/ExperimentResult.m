classdef ExperimentResult

    properties
        scenario % scenario of simulation
        options % config of the simulation
        iteration_data % iteration steps
        mpa % MotionPrimitiveAutomation used
        trajectory_predictions % predicted trajectory for all vehicles and iteration steps
        output_path % output_path of the ExperimentResult file
        total_fallback_times % total times of fallback

        %timings
        timings_general
        timings_per_vehicle

        %hlc info parts
        vehicle_path_fullres % path of all vehicles in iteration steps, TODO: can be optimized
        n_expanded % number of expansions in search tree during graph search
        vehicles_fallback % which vehicles should use their fallback trajectories
        computation_levels % TODO: should be calculated from directed_coupling ,scalar

        n_steps % total number of steps
        t_total % total runtime
    end

    methods

        function obj = ExperimentResult(options, scenario, mpa)

            arguments
                options (1, 1) Config;
                scenario (1, 1) Scenario;
                mpa (1, 1) MotionPrimitiveAutomaton;
            end

            obj.scenario = scenario;
            obj.options = options;
            obj.iteration_data = cell(0, 1);
            obj.mpa = mpa;
            obj.trajectory_predictions = cell(options.amount, 0);
            obj.output_path = '';
            obj.total_fallback_times = 0;

            obj.timings_general = cell(0, 1);
            obj.timings_per_vehicle = cell(0, 1);

            obj.vehicle_path_fullres = cell(options.amount, 0);
            obj.n_expanded = zeros(options.amount, 0);
            obj.vehicles_fallback = cell(0, 1);
            obj.computation_levels = zeros(1, 0);

            obj.n_steps = 0;
            obj.t_total = 0;
        end

    end

end
