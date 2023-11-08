classdef CouplerFactory

    methods (Static)

        function coupler = get_coupler(coupler_type)
            %GET_COUPLER creates a coupler according to the set option
            arguments
                coupler_type CouplerType;
            end

            switch (coupler_type)
                case CouplerType.ReachableSet
                    coupler = ReachableSetCoupler();
                case CouplerType.FullyConnected
                    coupler = FullyConnectedCoupler();
            end

        end

    end

end
