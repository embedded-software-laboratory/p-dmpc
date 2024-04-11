classdef Config

    properties
        % ----
        % Scenario
        scenario_type ScenarioType = ScenarioType.commonroad;
        scenario_file (1, :) char = 'scenario.mat'; % file the scenario is loaded from
        amount = 20; % integer, number of vehicles, does not include manual vehicles
        T_end = 20; % scalar, simulation duration
        path_ids (1, :) double = []; % reference path IDs for selection of paths for the vehicles
        start_poses (3, :) double; % initial poses of the vehicles (x, y, yaw) x n_vehicles

        % ----
        % Environment
        environment Environment = Environment.Simulation;
        options_plot_online OptionsPlotOnline = OptionsPlotOnline();

        computation_mode ComputationMode = ComputationMode.sequential;

        % ----
        % High-Level Controller
        is_prioritized = true; % true/false, is prioritize vehicles
        coupling CouplingStrategies = CouplingStrategies.reachable_set_coupling;
        priority PriorityStrategies = PriorityStrategies.constant_priority;
        weight WeightStrategies = WeightStrategies.distance_weight;
        cut CutStrategies = CutStrategies.greedy_cut;

        max_num_CLs = 99; % integer, maximum allowed number of computation levels

        optimizer_type OptimizerType = OptimizerType.MatlabOptimal; % optimizer that shall be used

        dt_seconds = 0.2; % scalar, default sample time
        Hp = 6; % scalar, prediction horizon

        mpa_type MpaType = MpaType.single_speed;

        constraint_from_successor ConstraintFromSuccessor = ConstraintFromSuccessor.area_of_standstill;

        manual_control_config ManualControlConfig = ManualControlConfig(); % manual control config

        should_do_dry_run (1, 1) logical = false;

        % ----
        % Other
        isDealPredictionInconsistency = true; % true/false, if true, reachability analysis will be used to deal with the problem of prediction inconsistency; otherwise, one-step delayed trajectories will be considered

        recursive_feasibility = true; % true/false, if true, the last trim must be an equilibrium trims
        time_per_tick = 0.01;
        offset = 0.01;
        is_use_dynamic_programming = true; % true/false, use dynamic programming or brute-force approach to calculate local reachable sets
    end

    properties (Dependent, GetAccess = public, SetAccess = private)
        tick_per_step; % number of data points per step
        k_end; % total number of steps
        % whether to allow non-convex polygons; the separating axis theorem
        % works only for convex polygons.
        are_any_obstacles_non_convex;
        % execute mex graph search functions in own process
        mex_out_of_process_execution
        use_cpp % whether to use the C++ optimizer
    end

    methods

        function result = get.tick_per_step(obj)
            result = round(obj.dt_seconds / obj.time_per_tick);
        end

        function result = get.k_end(obj)
            result = floor(obj.T_end / obj.dt_seconds);
        end

        function result = get.are_any_obstacles_non_convex(obj)

            if (obj.scenario_type == ScenarioType.circle ...
                    || ~obj.is_prioritized)
                % InterX only checks intersections of line segments. In the circle
                % scenario, reachable sets are large and therefore checking the
                % intersection of line segments is not sufficient to guarantee
                % avoidance of reachable sets in parallel planning.
                result = false;
            else
                % In road scenarios, reachable sets are always non-convex.
                % The separating axis theorem does not work, the error we make
                % with InterX is negligible due to small sizes of reachable sets.
                result = true;
            end

        end

        function mex_out_of_process_execution = get.mex_out_of_process_execution(obj)
            % if prioritized controller runs sequentially with more than 1 vehicle
            % activate out of process execution for mex function
            if obj.amount > 1 ...
                    && obj.is_prioritized ...
                    && obj.computation_mode == ComputationMode.sequential
                mex_out_of_process_execution = true;
            else
                mex_out_of_process_execution = false;
            end

        end

        function result = get.use_cpp(obj)
            result = (obj.optimizer_type == OptimizerType.CppOptimal | obj.optimizer_type == OptimizerType.CppSampled);
        end

        % empty set methods used by jsondecode
        % dependent properties with public GetAccess are encoded to a json file
        % to automatically decode the json file set methods must be defined

        function obj = set.tick_per_step(obj, ~)
        end

        function obj = set.k_end(obj, ~)
        end

        function obj = set.are_any_obstacles_non_convex(obj, ~)
        end

        function obj = set.mex_out_of_process_execution(obj, ~)
        end

        function obj = set.use_cpp(obj, ~)
        end

    end

    methods (Static)

        function obj = load_from_file(json_file_path)

            [~, ~, file_extension] = fileparts(json_file_path);

            assert( ...
                strcmp(file_extension, '.json'), ...
                'Input must be a json file!' ...
            );

            obj = Config();
            obj = jsondecode(obj, jsondecode(fileread(json_file_path)));

        end

    end

    methods (Access = public)

        function obj = Config()
        end

        function path_ids = randomize_path_ids(obj, optional)

            arguments
                obj (1, 1) Config = Config()
                optional.seed double = []
                optional.enforce_crossing_intersection (1, 1) logical = true
            end

            path_id_max = 41; % maximum defined path id

            if optional.enforce_crossing_intersection
                % the first 8 paths are on the outer circle
                possible_path_ids = 9:path_id_max;
            else
                possible_path_ids = 1:path_id_max; % all possible path ids
            end

            if isempty(optional.seed)
                random_stream = RandStream('mt19937ar');
            else
                random_stream = RandStream('mt19937ar', Seed = optional.seed);
            end

            path_ids = sort(randsample(random_stream, possible_path_ids, obj.amount), 'ascend');

        end

        function save_to_file(obj, optional)

            arguments
                obj (1, 1) Config
                optional.file_name (1, :) char = 'Config.json'
            end

            % Write Config to disk

            json_char = jsonencode(obj, PrettyPrint = true);

            fid = fopen(optional.file_name, 'w');
            fprintf(fid, json_char);
            fclose(fid);
        end

        function obj = jsondecode(obj, json_struct)
            % custom classes must provide jsondecode
            % for each loop requires fields as row vector
            fields = string(fieldnames(json_struct)).';

            for field = fields

                if ~isprop(obj, field)
                    warning('Cannot set property %s for class Config as it does not exist', field);
                    continue;
                end

                % custom classes must provide jsondecode
                if strcmp(field, "manual_control_config")
                    obj.manual_control_config = ManualControlConfig();
                    obj.manual_control_config = obj.manual_control_config.jsondecode(json_struct.manual_control_config);
                elseif strcmp(field, "options_plot_online")
                    obj.options_plot_online = OptionsPlotOnline();
                    obj.options_plot_online = obj.options_plot_online.jsondecode(json_struct.options_plot_online);
                else
                    obj.(field) = json_struct.(field);
                end

            end

        end

        function obj = validate(obj)

            % throw error if centralized controller should run in lab
            if ( ...
                    obj.environment == Environment.CpmLab || ...
                    obj.environment == Environment.UnifiedTestbedInterface ...
                )
                assert( ...
                    isunix && ~ismac, ...
                    'The lab interfaces can only be used on a Linux system!' ...
                )
                assert( ...
                    obj.is_prioritized == true, ...
                    'You are trying to run a centralized controller in the lab!' ...
                )
            end

            % limit maximum number of computation levels
            obj.max_num_CLs = min(obj.max_num_CLs, obj.amount);

            if obj.scenario_type ~= ScenarioType.circle
                % set default path ids if no path ids were defined
                if isempty(obj.path_ids)

                    switch obj.amount
                        case 1
                            obj.path_ids = 18;
                        case 2
                            obj.path_ids = [18, 20];
                        case 3
                            obj.path_ids = [18, 19, 20];
                        case 4
                            obj.path_ids = [17, 18, 19, 20];
                        otherwise
                            obj.path_ids = obj.randomize_path_ids();
                    end

                end

                % validate amount of path ids
                assert( ...
                    length(obj.path_ids) == obj.amount, ...
                    'Amount of path_ids (%d) does not match amount of vehicles (%d)!', ...
                    length(obj.path_ids), ...
                    obj.amount ...
                )

                % validate that path_ids are unique
                assert( ...
                    length(obj.path_ids) == length(unique(obj.path_ids, 'stable')), ...
                    'Path_ids must be unique!' ...
                );
            end

            % validate manual control config
            if ~obj.manual_control_config.is_active
                % set values for inactive manual control config
                obj.manual_control_config.amount = 0;
                obj.manual_control_config.hdv_ids = [];
            else
                % check if chosen amount matches with typed in hdv ids
                assert( ...
                    length(obj.manual_control_config.hdv_ids) == obj.manual_control_config.amount, ...
                    'Amount of hdv_ids (%d) does not match amount of manual vehicles (%d)!', ...
                    length(obj.manual_control_config.hdv_ids), ...
                    obj.manual_control_config.amount ...
                )
            end

        end

        function tf = isequal(obj, other_config)

            arguments
                obj (1, 1) Config
                other_config (1, 1) Config
            end

            % check if two configs are equal
            tf = true;

            % check if all properties are equal
            all_properties = string(fieldnames(obj)).';

            irrelevant_properties = [
                                     "time_per_tick"
                                     "is_use_dynamic_programming"
                                     "options_plot_online"
                                     ];

            for property = all_properties

                if ismember(property, irrelevant_properties)
                    continue;
                end

                if ~isequal(obj.(property), other_config.(property))
                    tf = false;
                    return;
                end

            end

        end

    end

end
