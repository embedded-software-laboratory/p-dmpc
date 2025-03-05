function write_scenarios_from_experiment_results(optional)

    arguments
        optional.computation_mode (1, 1) ComputationMode = ComputationMode.parallel_physically
        optional.scenario (1, 1) ScenarioType = ScenarioType.commonroad
        optional.optimizer (1, 1) OptimizerType = OptimizerType.MatlabSampled
        optional.priority_strategy (1, 1) PriorityStrategies = PriorityStrategies.constant_priority
        optional.Hp (1, 1) double = 6;
        optional.base_folder string = fullfile(FileNameConstructor.all_results(), 'phd')
    end

    % vehicles x approaches x seeds
    experiment_results = eval_experiments( ...
        computation_mode = optional.computation_mode, ...
        scenario_type = optional.scenario, ...
        optimizer = optional.optimizer, ...
        priority_strategies = optional.priority_strategy, ...
        Hp = optional.Hp ...
    );

    n_vehicles = experiment_results(3, 1, 1).options.amount;
    vehicle_table = table( ...
        Size = [n_vehicles 6], ...
        VariableTypes = repmat("double", [1, 6]), ...
        VariableNames = ["id", "xStart", "yStart", "psiStart", "vRef", "pathId"] ...
    );

    path_x_y = cell(6, 1);
    % loop over all seeds for 15 vehicles
    for i_seed = 1:size(experiment_results, 3)
        experiment_result = experiment_results(3, 1, i_seed);
        scenario = Scenario.create(experiment_result.options);

        for i_vehicle = 1:n_vehicles
            vehicle_table.id(i_vehicle) = i_vehicle;
            vehicle_table.xStart(i_vehicle) = scenario.vehicles(i_vehicle).x_start;
            vehicle_table.yStart(i_vehicle) = scenario.vehicles(i_vehicle).y_start;
            vehicle_table.psiStart(i_vehicle) = scenario.vehicles(i_vehicle).yaw_start;
            vehicle_table.vRef(i_vehicle) = scenario.vehicles(i_vehicle).reference_speed;

            % find unique paths
            path_id = experiment_result.options.path_ids(i_vehicle);

            if (path_id <= 13)

                if isempty(path_x_y{1})
                    path_x_y{1} = scenario.vehicles(i_vehicle).reference_path;
                end

                vehicle_table.pathId(i_vehicle) = 1;

            elseif (path_id <= 18)

                if isempty(path_x_y{2})
                    path_x_y{2} = scenario.vehicles(i_vehicle).reference_path;
                end

                vehicle_table.pathId(i_vehicle) = 2;

            elseif (path_id <= 23)

                if isempty(path_x_y{3})
                    path_x_y{3} = scenario.vehicles(i_vehicle).reference_path;
                end

                vehicle_table.pathId(i_vehicle) = 3;

            elseif (path_id <= 28)

                if isempty(path_x_y{4})
                    path_x_y{4} = scenario.vehicles(i_vehicle).reference_path;
                end

                vehicle_table.pathId(i_vehicle) = 4;

            elseif (path_id <= 34 || path_id == 41)

                if isempty(path_x_y{5})
                    path_x_y{5} = scenario.vehicles(i_vehicle).reference_path;
                end

                vehicle_table.pathId(i_vehicle) = 5;

            elseif (path_id <= 40)

                if isempty(path_x_y{6})
                    path_x_y{6} = scenario.vehicles(i_vehicle).reference_path;
                end

                vehicle_table.pathId(i_vehicle) = 6;

            end

        end

        writetable(vehicle_table, fullfile(optional.base_folder, sprintf('5-15-road-%i.dat', i_seed)));

    end

    max_rows = max(cellfun("size", path_x_y, 1));
    variable_names = [string("x" + (1:6)), string("y" + (1:6))];
    path_table = table( ...
        Size = [max_rows, 2 * numel(path_x_y)], ...
        VariableTypes = repmat("doublenan", [1, 2 * numel(path_x_y)]), ...
        VariableNames = variable_names ...
    );

    for i = 1:numel(path_x_y)
        x = path_x_y{i}(:, 1);
        x(end + 1:max_rows) = NaN;
        y = path_x_y{i}(:, 2);
        y(end:max_rows) = NaN;
        path_table(:, i) = table(x);
        path_table(:, i + 6) = table(y);
    end

    writetable(path_table, fullfile(optional.base_folder, '5-15-road-paths.dat'));

end
