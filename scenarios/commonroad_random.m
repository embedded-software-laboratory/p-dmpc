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
    options_copy = copy(options);

    for iVeh = 1:length(nVeh)

        for iSeed = 1:length(seed)
            options = copy(options_copy);
            options.amount = nVeh(iVeh);
            random_stream = RandStream('mt19937ar', 'Seed', seed(iSeed));
            path_ids = sort(randsample(random_stream, 1:40, options.amount), 'ascend');
            options.path_ids = path_ids;
            scenario = commonroad(options, options.path_ids);
            scenario.random_stream = random_stream;

            for idVeh = 1:options.amount
                % initialize vehicle ids of all vehicles
                scenario.vehicles(idVeh).ID = path_ids(idVeh);
            end

            scenarios(iVeh, iSeed) = scenario;
        end

    end

end
