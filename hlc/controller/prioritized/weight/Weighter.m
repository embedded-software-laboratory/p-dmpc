classdef (Abstract) Weighter < handle

    methods (Abstract)
        weigh(obj, scenario, mpa, iter)
    end

    methods (Static)

        function weighter = get_weighter(method)
            % GET_WEIGHTER  Given a weighting method this function returns the corresponding weighter.

            switch method
                case WeightStrategies.constant_weight
                    weighter = ConstantWeighter();
                case WeightStrategies.random_weight
                    weighter = RandomWeighter();
                case WeightStrategies.STAC_weight
                    weighter = StacWeighter();
                case WeightStrategies.optimal_weight
                    weighter = OptimalWeighter();
                case WeightStrategies.distance_weight
                    weighter = DistanceWeighter();
                otherwise
                    weighter = ConstantWeighter();
                    warning('Unavailable Weighting Method chosen. Using Constant Weight.');
            end

        end

    end

end
