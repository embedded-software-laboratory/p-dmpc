function scenarios = commonroad_random(options, amounts, seeds)
    % commonroad_random - generate multiple options for commonroad scenarios
    % with random path_ids based on passed seeds and passed amounts
    %
    % Output:
    %   scenarios (n_amounts, n_seeds) Scenario

    arguments
        options (1, 1) Config
        amounts (1, :) double
        seeds (1, :) double
    end

    disp('Creating scenarios...')
    scenarios(length(amounts), length(seeds)) = Scenario();

    for iVeh = 1:length(amounts)

        for iSeed = 1:length(seeds)
            options_random = options;
            options_random.amount = amounts(iVeh);
            random_stream = RandStream('mt19937ar', 'Seed', seeds(iSeed));
            path_ids = sort(randsample(random_stream, 1:40, options_random.amount), 'ascend');
            options_random.path_ids = path_ids;
            scenario = commonroad_scenario(options_random.amount, options_random.path_ids);

            % FIXME this property is not supported anymore
            scenario.random_stream = random_stream;

            scenarios(iVeh, iSeed) = scenario;
        end

    end

end
