classdef ControlResultsInfo
    % CONTROLRESULTSINFO Summary of this class goes here
    %   Detailed explanation goes here

    properties
        tree (:, 1) % array of trees
        tree_path (:, :) double % tree path
        n_expanded (:, 1) double % number of times that nodes are expended during graph search
        next_node % next node information
        shapes (:, :) cell % predicted occupied areas of all prediction horizons
        predicted_trims (:, :) double % predicted trims of all prediction horizon (including the current trim)
        trim_indices (:, 1) double % predicted trim of the next time step
        y_predicted (:, 1) cell % predicted trajectory
        computation_levels (1, 1) double % actual number of computation levels of the whole system
        vehicles_fallback (:, 1) int32 % vehicles that need to take fallback
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
            obj.predicted_trims = zeros(nVeh, Hp + 1);
            obj.y_predicted = cell(nVeh, 1);
            obj.computation_levels = inf;
            obj.vehicles_fallback = int32.empty;
            obj.is_exhausted = false(nVeh, 1);
            obj.needs_fallback = false(nVeh, 1);
            %obj.u = zeros(nVeh,1);
        end

        function obj = store_control_info(obj, info_v, options)

            if options.is_prioritized
                i_vehicle = find(obj.controller_ID == info_v.controller_ID);
                obj.tree{i_vehicle} = info_v.tree;
                obj.tree_path(i_vehicle, :) = info_v.tree_path;
                obj.n_expanded(i_vehicle, 1) = info_v.n_expanded;
                obj.next_node = set_node(obj.next_node, i_vehicle, info_v.tree.get_node(info_v.tree_path(2)));
                obj.shapes(i_vehicle, :) = info_v.shapes(:);
                obj.predicted_trims(i_vehicle, :) = info_v.predicted_trims; % store the planned trims in the future Hp time steps
                obj.y_predicted(i_vehicle) = info_v.y_predicted; % store the information of the predicted output
                obj.is_exhausted(i_vehicle) = info_v.is_exhausted;

            else
                % for centralized control
                obj.tree = info_v.tree; % only for node explorationslee
                obj.n_expanded = info_v.n_expanded;
                obj.next_node = set_node(obj.next_node, 1:options.amount, info_v.tree.get_node(info_v.tree_path(2)));
                obj.shapes = info_v.shapes;
                obj.predicted_trims = info_v.predicted_trims; % store the planned trims in the future Hp time steps
                obj.y_predicted = info_v.y_predicted(:); % store the information of the predicted output
                obj.is_exhausted = info_v.is_exhausted;
            end

            % Predicted trim of the next time step
            obj.trim_indices = obj.predicted_trims(:, 2);
        end

    end

end
