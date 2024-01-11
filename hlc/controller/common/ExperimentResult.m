classdef ExperimentResult

    properties
        scenario Scenario % scenario of simulation
        options Config % config of the simulation
        mpa MotionPrimitiveAutomaton % MotionPrimitiveAutomation used

        iteration_data IterationData % iteration steps

        control_results_info ControlResultsInfo
        trajectory_predictions % predicted trajectory for all vehicles and iteration steps
        output_path % output_path of the ExperimentResult file
        total_fallback_times % total times of fallback

        %hlc info parts
        n_expanded % number of expansions in search tree during graph search
        vehicles_fallback % which vehicles should use their fallback trajectories
        computation_levels % TODO: should be calculated from directed_coupling ,scalar

        n_steps % total number of steps
        t_total % total runtime

        timing

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
            obj.mpa = mpa;
            obj.trajectory_predictions = cell(options.amount, 0);
            obj.output_path = '';
            obj.total_fallback_times = 0;

            obj.timing = cell(0, 1);

            obj.n_expanded = zeros(options.amount, 0);
            obj.vehicles_fallback = cell(0, 1);
            obj.computation_levels = zeros(1, 0);

            obj.n_steps = 0;
            obj.t_total = 0;
        end

        function equal = is_equal(obj, compare_obj)

            arguments
                obj (1, 1) ExperimentResult;
                compare_obj (1, 1) ExperimentResult;
            end

            equal = obj.n_steps == compare_obj.n_steps;

            for i_step = 1:obj.n_steps
                equal = equal && obj.iteration_data{i_step}.is_equal(compare_obj.iteration_data{i_step});

                if (~equal)
                    return;
                end

            end

        end

    end

end
