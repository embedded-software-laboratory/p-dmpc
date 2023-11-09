classdef StacWeigher < Weigher
    % STACWEIGHER  Instance of weight used for coupling weighing
    % weight based on shortest time to achieve collision (STAC)

    properties (Access = private)
    end

    properties (Constant, Access = private)
        STAC_threshold = 0.5; % vehicles are considered as very close if they can achieve a collision in less than this time
        sensitive_factor = 1; % sensitive factor used to calculate the coupling weights. The bigger, the more sensitive to the STAC. Default value 1.
        side_impact_weight_scale_factor = 1.25; % side-impact collision is harder to avoid, so we scale its weight
    end

    methods

        function obj = StacWeigher()
        end

        function [weighted_coupling] = weigh(obj, iter, time_step, options, ~)
            weighted_coupling = iter.directed_coupling;

            if ((time_step == 1) && ~(options.scenario_type == ScenarioType.commonroad))
                warning('STAC Weight only available for Commonroad scenarios! May behave unexpectedly!')
            end

            [rows, cols] = find(iter.directed_coupling);

            for i_row_col_pair = 1:length(rows)
                row = rows(i_row_col_pair);
                col = cols(i_row_col_pair);
                stac = iter.coupling_info{row, col}.stac;
                weighted_coupling(row, col) = obj.weighting_function(stac, obj.sensitive_factor);
            end

        end

    end

    methods (Access = private)

        function coupling_weight = weighting_function(~, STAC, sensitive_factor)
            % returns the coupling weight based on the shortest time to achieve a
            % collision (STAC)
            if nargin == 1
                sensitive_factor = 1; % default value 1
            end

            coupling_weight = exp(-sensitive_factor * STAC); % negative exponential function
        end

    end

end
