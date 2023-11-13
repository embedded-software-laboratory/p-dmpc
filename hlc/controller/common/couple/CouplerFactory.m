classdef CouplerFactory

    methods (Static)

        function coupler = get_coupler(coupling)
            %GET_COUPLER creates a coupler according to the set option
            arguments
                coupling CouplerStrategies;
            end

            switch (coupling)
                case CouplerStrategies.reachable_set_coupling
                    coupler = ReachableSetCoupler();
                case CouplerStrategies.full_coupling
                    coupler = FullyConnectedCoupler();
            end

        end

    end

end
