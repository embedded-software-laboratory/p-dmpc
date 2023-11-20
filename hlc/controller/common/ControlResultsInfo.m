classdef ControlResultsInfo
    % CONTROLRESULTSINFO Summary of this class goes here
    %   Detailed explanation goes here

    properties
        tree (:, 1) % array of trees
        tree_path (:, :) double % tree path
        n_expanded (:, 1) double % number of times that nodes are expended during graph search
        next_node % next node information
        shapes (:, :) cell % predicted occupied areas of all prediction horizons
        vehicle_fullres_path (:, 1) cell % predicted trajectory of the next time step
        predicted_trims (:, :) double % predicted trims of all prediction horizon (including the current trim)
        trim_indices (:, 1) double % predicted trim of the next time step
        y_predicted (:, 1) cell % predicted trajectory
        computation_levels (1, 1) double % actual number of computation levels of the whole system
        vehs_fallback (:, 1) int32 % vehicles that need to take fallback
        is_exhausted (:, 1) logical % whether graph search is exhausted
        needs_fallback % vehicle at a stillstand but the graph search is still exhausted
        u (:, 1) double % control input
    end

    properties (Dependent)

    end

    properties (SetAccess = private)
        controller_ID % controller ID which should be the same as the corresponding vehicle ID
    end

    methods

        function obj = ControlResultsInfo(nVeh, Hp, controller_ID)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            if nargin == 2
                obj.controller_ID = 1:nVeh; % default controller ID
                warning('Default controller ID is used.')
            else
                obj.controller_ID = controller_ID;
            end

            assert(length(obj.controller_ID) == nVeh)

            obj.tree = cell(nVeh, 1);
            obj.tree_path = zeros(nVeh, Hp + 1);
            obj.n_expanded = zeros(nVeh, 1);
            obj.next_node = node(-1, zeros(nVeh, 1), zeros(nVeh, 1), zeros(nVeh, 1), zeros(nVeh, 1), -1, -1);
            obj.shapes = cell(nVeh, Hp);
            obj.vehicle_fullres_path = cell(nVeh, 1); %TODO not used
            obj.predicted_trims = zeros(nVeh, Hp + 1);
            obj.y_predicted = cell(nVeh, 1);
            obj.computation_levels = inf;
            obj.vehs_fallback = int32.empty;
            obj.is_exhausted = false(nVeh, 1);
            obj.needs_fallback = false(nVeh, 1);
            %obj.u = zeros(nVeh,1);
        end

        function obj = store_control_info(obj, info_v, options, mpa)
            % Store the control information, such as `tree`, `tree_path`,
            % `n_expanded`, `next_node`, `shapes`, `vehicle_fullres_path`, `predicted_trims`, `y_predicted`
            if options.is_prioritized
                vehicle_idx = find(obj.controller_ID == info_v.controller_ID);
                obj.tree{vehicle_idx} = info_v.tree;
                obj.tree_path(vehicle_idx, :) = info_v.tree_path;
                obj.n_expanded(vehicle_idx, 1) = info_v.n_expanded;
                obj.next_node = set_node(obj.next_node, vehicle_idx, info_v.tree.get_node(info_v.tree_path(2)));
                obj.shapes(vehicle_idx, :) = info_v.shapes(:);
                obj.vehicle_fullres_path(vehicle_idx) = path_between(info_v.tree_path(1), info_v.tree_path(2), info_v.tree, mpa);
                obj.predicted_trims(vehicle_idx, :) = info_v.predicted_trims; % store the planned trims in the future Hp time steps
                %             obj.trim_indices(vehicle_idx) = info_v.trim_indices; % dependent variable
                obj.y_predicted(vehicle_idx) = info_v.y_predicted; % store the information of the predicted output
                obj.is_exhausted(vehicle_idx) = info_v.is_exhausted;

            else
                % for centralized control
                obj.tree = info_v.tree; % only for node explorationslee
                obj.n_expanded = info_v.n_expanded;
                obj.next_node = set_node(obj.next_node, 1:options.amount, info_v.tree.get_node(info_v.tree_path(2)));
                obj.shapes = info_v.shapes;
                obj.vehicle_fullres_path = path_between(info_v.tree_path(1), info_v.tree_path(2), info_v.tree, mpa)';
                obj.predicted_trims = info_v.predicted_trims; % store the planned trims in the future Hp time steps
                %                     obj.trim_indices = info_v.trim_indices; % dependent variable
                obj.y_predicted = info_v.y_predicted(:); % store the information of the predicted output
                obj.is_exhausted = info_v.is_exhausted;
            end

            % Predicted trim of the next time step
            obj.trim_indices = obj.predicted_trims(:, 2);
        end

        function obj = handle_graph_search_exhaustion(obj, options, iter, mpa)
            trim = iter.trim_indices;

            if mpa.trims(trim).speed == 0 && ~strcmp(options.strategy_consider_veh_without_ROW, '1')
                % if a vehicle at a standstill cannot find a feasible
                % trajectory, it will keep at a standstill without
                % triggering a fallback. This kind of graph search is
                % considered as a semi-exhausted, and the corresponding
                % infeasibility is called a semi-infeasibility. Note that
                % this strategy can be used only if higher-priority
                % vehicles consider at least the current occupied sets of
                % lower-priority vehicles.
                Hp = options.Hp;
                x = iter.x0(:, 1);
                y = iter.x0(:, 2);
                yaw = iter.x0(:, 3);
                obj.tree_path = ones(size(x, 1), Hp + 1);
                y_pred = {repmat([x, y, yaw, trim], (options.tick_per_step + 1) * Hp, 1)};
                obj.y_predicted = y_pred;

                vehiclePolygon = transformed_rectangle(x, y, yaw, iter.vehicles.Length, iter.vehicles.Width);
                shape_veh = {[vehiclePolygon, vehiclePolygon(:, 1)]}; % close shape

                obj.shapes = repmat(shape_veh, 1, Hp);
                % Predicted trims in the future Hp time steps. The first entry is the current trims
                obj.predicted_trims = repmat(trim, 1, Hp + 1);
                obj.needs_fallback = false;
            else
                obj.needs_fallback = true;
            end

        end

    end

end
