function scenarios = commonroad_random(options, nVeh, seed)
    % commonroad_random - generate a random scenario
    %   scenarios = commonroad_random(options, nVeh, seed)
    %  options: OptionsMain object (vehicle ids are ignored)
    %  nVeh: number of vehicles (can be array)
    %  seed: random seed (can be array)
    arguments
        options (1, 1) Config
        nVeh (1, :) double
        seed (1, :) double
    end

    disp('Creating scenarios...')
    scenarios(length(nVeh), length(seed)) = Scenario();

    for iVeh = 1:length(nVeh)

        for iSeed = 1:length(seed)
            options_random = options;
            options_random.amount = nVeh(iVeh);
            random_stream = RandStream('mt19937ar', 'Seed', seed(iSeed));
            path_ids = sort(randsample(random_stream, 1:40, options_random.amount), 'ascend');
            options_random.path_ids = path_ids;
            scenario = commonroad(options_random.amount, options_random.path_ids);
            scenario.options = options_random;

            % FIXME this property is not supported anymore
            scenario.random_stream = random_stream;

            scenarios(iVeh, iSeed) = scenario;
        end

    end

end
