classdef ControlResultsInfo
    % CONTROLRESULTSINFO Summary of this class goes here
    %   Detailed explanation goes here

    properties
        tree % (1, 1) Tree
        % array of tree vertices that form the solution, (1 x prediction horizon)
        tree_path (1, :) double
        n_expanded (1, 1) double = 0 % number of expanded nodes in the tree
        shapes (:, :) cell % occupied areas, n_vehicles x prediction horizons
        % predicted trims, n_vehicles x prediction horizon
        predicted_trims (:, :) double
        % predicted trajectory, (x, y, yaw) x prediction horizon x n_vehicles
        y_predicted (3, :, :) double
        is_exhausted (1, 1) logical = false % whether graph search is exhausted
        needs_fallback (1, 1) logical = false %
    end

    properties (Dependent)

        n_vehicles
        prediction_horizon

    end

    methods

        function result = get.n_vehicles(obj)
            result = size(obj.y_predicted, 3);
        end

        function result = get.prediction_horizon(obj)
            result = size(obj.y_predicted, 2);
        end

        function obj = ControlResultsInfo(nVeh, Hp)
            obj.tree_path = zeros(1, Hp + 1);
            obj.shapes = cell(nVeh, Hp);
            obj.predicted_trims = zeros(nVeh, Hp);
            obj.y_predicted = nan(3, Hp, nVeh);
        end

    end

    methods (Static)

        function cleaned_object = clean(object)

            arguments
                object (1, 1) ControlResultsInfo
            end

            cleaned_object = object;
            cleaned_object.tree = [];
            cleaned_object.tree_path = [];
            cleaned_object.shapes = [];
        end

    end

end
