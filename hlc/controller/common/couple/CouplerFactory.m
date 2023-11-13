classdef CouplerFactory

    methods (Static)

        function coupler = get_coupler(coupler_type)
            %GET_COUPLER creates a coupler according to the set option
            arguments
                coupler_type CouplerStrategies;
            end

            switch (coupler_type)
                case CouplerStrategies.ReachableSet
                    coupler = ReachableSetCoupler();
                case CouplerStrategies.FullyConnected
                    coupler = FullyConnectedCoupler();
            end

        end

    end

end
