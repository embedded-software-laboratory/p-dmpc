classdef (Abstract) Coupler < handle

    properties (Access = protected)
        intersection_ids
    end

    properties (Constant, Access = protected)
        intersection_distance_threshold = 1.2; % vehicles are considered as at the intersection if their distances to the center point of intersection is smaller than this value
    end

    methods (Abstract)

        % Returns the adjacency matrix
        [adjacency] = couple(obj, options, max_mpa_speed, iter)

    end

    methods (Static)

        function coupler = get_coupler(coupling, amount, scenario)
            %GET_COUPLER creates a coupler according to the set option
            arguments
                coupling CouplingStrategies;
                amount double; % options.amount for constant coupling
                scenario Scenario;
            end

            switch (coupling)
                case CouplingStrategies.reachable_set_coupling
                    coupler = ReachableSetCoupler();
                case CouplingStrategies.full_coupling
                    coupler = ConstantCoupler(ones(amount) - eye(amount));
                case CouplingStrategies.no_coupling
                    coupler = ConstantCoupler(zeros(amount));
                case CouplingStrategies.distance_coupling
                    coupler = DistanceCoupler(scenario);
            end

        end

    end

    methods

        function obj = Coupler()
        end

    end

end
