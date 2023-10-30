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
                case WeightStrategies.distance_weight
                    weigher = DistanceWeigher();
            end

        end

    end

end
