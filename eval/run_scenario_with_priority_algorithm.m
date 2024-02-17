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

                if FileNameConstructor.result_exists(scenarios(iVeh, iSeed).options)
                    disp('File already exists.')
                    r = load_latest(scenarios(iVeh, iSeed).options);
                    experiment_result = r.experiment_result;
                else
                    % run simulation
                    % FIXME this will not work after the options are removed from the scenario object
                    experiment_result = main(scenarios(iVeh, iSeed));
                end

                results{iVeh, i_priority, iSeed} = experiment_result;

                % evaluate
                %e_differentNumVehs{i_priority} = EvaluationParl(options,[0,options.T_end]);

                % display progress
                count = count + 1;
                disp(['--------Progress ' num2str(count) '/' num2str(n_simulations) ': done--------'])
            end

        end

    end

end
