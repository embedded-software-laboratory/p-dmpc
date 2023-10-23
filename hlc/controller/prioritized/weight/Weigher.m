classdef (Abstract) Weigher < handle

    methods (Abstract)
        weigh(obj, scenario, mpa, iter)
    end

    methods (Static)

        function weigher = get_weigher(method)
            % GET_WEIGHER  Given a weighing method this function returns the corresponding weigher.

            switch method
                case WeightStrategies.constant_weight
                    weigher = ConstantWeigher();
                case WeightStrategies.random_weight
                    weigher = RandomWeigher();
                case WeightStrategies.STAC_weight
                    weigher = StacWeigher();
                case WeightStrategies.optimal_weight
                    weigher = OptimalWeigher();
                case WeightStrategies.distance_weight
                    weigher = DistanceWeigher();
                otherwise
                    weigher = ConstantWeigher();
                    warning('Unavailable Weighing Method chosen. Using Constant Weight.');
            end

        end

    end

end
