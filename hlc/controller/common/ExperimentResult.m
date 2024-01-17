classdef ExperimentResult

    properties
        scenario Scenario % scenario of simulation
        options Config % config of the simulation
        mpa MotionPrimitiveAutomaton % MotionPrimitiveAutomation used

        iteration_data IterationData % iteration steps

        control_results_info ControlResultsInfo
        output_path % output_path of the ExperimentResult file
        total_fallback_times % total times of fallback

        %hlc info parts
        n_hlc % number of hlcs whose experiment results are contained
        hlc_indices % indices of vehicles the hlc (belonging to this result) controls; scalar for distributed computation

        n_steps % total number of steps
        t_total % total runtime

        timing

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
            obj.total_fallback_times = 0;

            obj.timing = cell(0, 1);

            obj.n_steps = 0;
            obj.t_total = 0;
            obj.hlc_indices = hlc_indices;
            obj.n_hlc = length(hlc_indices);
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
