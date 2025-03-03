classdef BuiltScenario < ScenarioAdapter
    % BuiltScenario This scenario adapter loads a scenario
    % that was built from the disk

    properties (Access = public)
        scenario Scenario % (1, 1)
    end

    methods (Access = public)

        function obj = BuiltScenario()
            obj = obj@ScenarioAdapter();
        end

        function init(obj, options, plant)
            % initialize the member scenario of the scenario adapter
            % with a scenario saved in specified scenario file

            arguments
                obj (1, 1) BuiltScenario
                options (1, 1) Config
                plant (1, 1) Plant
            end

            scenario_loaded = load(options.scenario_file, 'scenario').scenario;

            assert( ...
                length(scenario_loaded.vehicles) == options.amount, ...
                'Amount of vehicles in scenario (%d) does not match amount (%d)', ...
                length(scenario_loaded.vehicles), ...
                options.amount ...
            )

            obj.scenario = scenario_loaded;

            % init manual vehicles
            obj.init_hdv(options);
        end

    end

    methods (Access = private)

        function register_map(~, map_as_string)
            % registering the map of the scenario
            % the functionality is currently part of UnifiedTestbedInterface
            fprintf('Successfully registered the map %s.', map_as_string);
        end

    end

end
