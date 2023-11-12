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

    path_id_max = 41; % maximum defined path id

    disp('Creating options and scenarios...')
    options_array(length(amounts), length(seeds)) = Config();
    scenario_array(length(amounts), length(seeds)) = Scenario();

    for i_amount = 1:length(amounts)

        for i_seed = 1:length(seeds)
            random_stream = RandStream('mt19937ar', 'Seed', seeds(i_seed));

            options_random = options;
            options_random.amount = amounts(i_amount);
            options_random.path_ids = randperm(random_stream, path_id_max, options_random.amount);

            options_array(i_amount, i_seed) = options_random;

            scenario = commonroad_scenario(options_random.amount, options_random.path_ids);

            % FIXME this property is not supported anymore
            scenario.random_stream = random_stream;

            scenario_array(i_amount, i_seed) = scenario;
        end

    end

end
