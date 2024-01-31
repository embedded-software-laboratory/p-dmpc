function [options_array, scenario_array] = commonroad_random(amounts, seeds, options, optional)
    % commonroad_random - generate multiple options with commonroad scenarios
    % with random path_ids based on passed seeds and passed amounts
    %
    % Output:
    %   options_array (n_amounts, n_seeds) Config
    %   scenario_array (n_amounts, n_seeds) Scenario

    arguments
        amounts (1, :) double
        seeds (1, :) double
        % options based on which the output arguments are created
        options (1, 1) Config = Config();
        optional.enforce_crossing_intersection (1, 1) logical = false
    end

    % validate passed options
    options = options.validate();

    assert( ...
        options.scenario_type == ScenarioType.commonroad, ...
        'Scenario type must be commonroad! Argument or default type is not correct!' ...
    )

    path_id_max = 41; % maximum defined path id
    possible_path_ids = 1:path_id_max; % all possible path ids

    if optional.enforce_crossing_intersection
        % the first 8 paths are on the outer circle
        path_ids_outer_circle = 1:8;
        % remove path ids for paths on the outer circle
        possible_path_ids = setdiff(possible_path_ids, path_ids_outer_circle);
    end

    fprintf('Creating options and if requested scenarios... ');
    options_array(length(amounts), length(seeds)) = Config();
    scenario_array(length(amounts), length(seeds)) = Scenario();

    for i_amount = 1:length(amounts)

        for i_seed = 1:length(seeds)
            random_stream = RandStream('mt19937ar', 'Seed', seeds(i_seed));

            options_random = options;
            options_random.amount = amounts(i_amount);
            options_random.path_ids = randsample(random_stream, possible_path_ids, options.amount);

            % validate modified options
            options_random = options_random.validate();

            options_array(i_amount, i_seed) = options_random;

            if nargout == 1
                % do not create scenarios if they were not requested
                continue
            end

            scenario = Commonroad(options_random.amount, options_random.path_ids);

            scenario_array(i_amount, i_seed) = scenario;
        end

    end

    fprintf('done\n');

end
