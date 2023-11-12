function [options_array, scenario_array] = commonroad_random(options, amounts, seeds)
    % commonroad_random - generate multiple options with commonroad scenarios
    % with random path_ids based on passed seeds and passed amounts
    %
    % Output:
    %   options_array (n_amounts, n_seeds) Config
    %   scenario_array (n_amounts, n_seeds) Scenario

    arguments
        options (1, 1) Config
        amounts (1, :) double
        seeds (1, :) double
    end

    disp('Creating options and scenarios...')
    options_array(length(amounts), length(seeds)) = Config();
    scenario_array(length(amounts), length(seeds)) = Scenario();

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

            scenario_array(iVeh, iSeed) = scenario;
        end

    end

end
