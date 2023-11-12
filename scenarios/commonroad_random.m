function scenarios = commonroad_random(options, amounts, seed)
    % commonroad_random - generate a random scenario
    %   scenarios = commonroad_random(options, amounts, seed)
    %  options: OptionsMain object (vehicle ids are ignored)
    %  amounts: number of vehicles (can be array)
    %  seed: random seed (can be array)
    arguments
        options (1, 1) Config
        amounts (1, :) double
        seed (1, :) double
    end

    disp('Creating scenarios...')
    scenarios(length(amounts), length(seed)) = Scenario();

    for iVeh = 1:length(amounts)

        for iSeed = 1:length(seed)
            options_random = options;
            options_random.amount = amounts(iVeh);
            random_stream = RandStream('mt19937ar', 'Seed', seed(iSeed));
            path_ids = sort(randsample(random_stream, 1:40, options_random.amount), 'ascend');
            options_random.path_ids = path_ids;
            scenario = commonroad_scenario(options_random.amount, options_random.path_ids);

            % FIXME this property is not supported anymore
            scenario.random_stream = random_stream;

            scenarios(iVeh, iSeed) = scenario;
        end

    end

end
