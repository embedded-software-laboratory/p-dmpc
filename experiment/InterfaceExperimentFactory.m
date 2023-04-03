classdef InterfaceExperimentFactory < handle
    %INTERFACEEXPERIMENTFACTORY Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % data queue for visualization. Used if running distributed HLCs
        % locally with Parallel Computing Toolbox
        visualization_data_queue
        % the created experimentInterface
        experimentInterface
        % store the used environment
        environment
    end
    
    methods
        function obj = InterfaceExperimentFactory()
            % Set default settings

            % Some default values are invalid and thus they're easily spotted when they haven't been explicitly set
            % We can then either throw an exception or use an arbitrary option when we find a default value
            % Or should we make valid and useful default values?
            obj.visualization_data_queue = [];
        end
        
        function experimentInterface = get_experiment_interface(obj,environment)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            obj.environment = environment;
            switch(environment)
                case Environment.CPMLab
                    experimentInterface = CPMLab();
                case Environment.Simulation
                    experimentInterface = SimLab(obj.visualization_data_queue);
                case Environment.UnifiedLabAPI
                    experimentInterface = UnifiedLabAPI();
            end
            obj.experimentInterface = experimentInterface;
        end

        function set_visualization_data_queue( obj )
            obj.visualization_data_queue = parallel.pool.DataQueue;
            if obj.environment == Environment.Simulation
                obj.experimentInterface.set_visualization_data_queue(obj.visualization_data_queue);
            end
        end
    end
end

