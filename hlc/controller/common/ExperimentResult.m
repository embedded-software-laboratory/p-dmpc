classdef ExperimentResult

    properties
        scenario Scenario % scenario of simulation
        options Config % config of the simulation
        mpa MotionPrimitiveAutomaton % MotionPrimitiveAutomation used

        iteration_data IterationData % iteration steps

        control_results_info ControlResultsInfo
        output_path % output_path of the ExperimentResult file

        %hlc info parts
        hlc_indices % indices of vehicles the hlc (belonging to this result) controls; scalar for distributed computation

        timing
    end

    properties (Dependent)
        n_steps % total number of steps
        t_total % total runtime
        n_hlc % number of hlcs whose experiment results are contained
    end

    methods

        function obj = ExperimentResult(options, scenario, mpa, hlc_indices)

            arguments
                options (1, 1) Config;
                scenario (1, 1) Scenario;
                mpa (1, 1) MotionPrimitiveAutomaton;
                hlc_indices (1, :) double;
            end

            obj.scenario = scenario;
            obj.options = options;
            obj.mpa = mpa;
            obj.output_path = '';

            obj.timing = cell(0, 1);
            obj.hlc_indices = hlc_indices;
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

        function value = get.n_steps(obj)
            value = length(obj.control_results_info);
        end

        function value = get.t_total(obj)
            value = obj.n_steps * obj.options.dt_seconds;
        end

        function value = get.n_hlc(obj);
            value = length(obj.hlc_indices);
        end

    end

end
