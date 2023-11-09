function results = run_scenario_with_priority_algorithm(scenarios, algorithm)

    arguments
        scenarios Scenario
        algorithm (1, :) string
    end

    [nVeh, nSeeds] = size(scenarios);
    results = cell(nVeh, length(algorithm), nSeeds);
    n_simulations = nVeh * nSeeds * length(algorithm);
    count = 0;
    disp('Starting simulations...')

    for iVeh = 1:nVeh
        disp(['# Vehicles: ', num2str(scenarios(iVeh, 1).options.amount)])

        for i_priority = 1:length(algorithm)
            disp(['Priority Assignment Algorithm: ', ...
                      algorithm{i_priority}] ...
            )

            for iSeed = 1:nSeeds
                scenarios(iVeh, iSeed).options.priority = ...
                    algorithm{i_priority};
                % run simulation
                % FIXME this will not work after the options are removed from the scenario object
                results_full_path = FileNameConstructor.get_results_full_path( ...
                    scenarios(iVeh, iSeed).options, ...
                    scenarios(iVeh, iSeed).options.path_ids ...
                );

                if isfile(results_full_path)
                    disp('File already exists.')
                    r = load(results_full_path);
                    result = r.result;
                else
                    % run simulation
                    % FIXME this will not work after the options are removed from the scenario object
                    result = main(scenarios(iVeh, iSeed));
                end

                results{iVeh, i_priority, iSeed} = clean_result(result);

                % evaluate
                %e_differentNumVehs{i_priority} = EvaluationParl(results_full_path,[0,options.T_end]);

                % display progress
                count = count + 1;
                disp(['--------Progress ' num2str(count) '/' num2str(n_simulations) ': done--------'])
            end

        end

    end

end

function result = clean_result(result_in)
    result.scenario = result_in.scenario;
    result.is_deadlock = result_in.is_deadlock;
    %result.priority = result_in.priority;
    result.t_total = result_in.t_total;
    result.nSteps = result_in.nSteps;
    result.output_path = result_in.output_path;
    result.iteration_structs = result_in.iteration_structs;
    result.directed_coupling = result_in.directed_coupling;
end
