classdef PlantFactory

    methods (Static)

        function plant = get_experiment_interface(environment)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            switch (environment)
                case Environment.CpmLab
                    plant = CpmLab();
                case Environment.Simulation
                    plant = SimLab();
                case Environment.SimulationDistributed
                    plant = SimLabDistributed();
                case Environment.UnifiedLabApi
                    plant = UnifiedLabApi();
            end

        end

    end

end
