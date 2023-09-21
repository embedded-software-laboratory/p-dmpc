classdef (Abstract) GraphSearchMex < OptimizerInterface

    properties (Access = protected)
        mexhosts = matlab.mex.MexHost.empty;
    end

    methods

        function obj = GraphSearchMex(scenario, veh_indices)
            obj = obj@OptimizerInterface(scenario);
            % When using C++, you don't want to send the scenario over
            % and over again, so it is done in the init function
            if obj.scenario.options.mex_out_of_process_execution
                % create mexhost for each vehicle (only if incremental search is used - no option in config yet)
                for i_veh = 1:length(veh_indices)
                    obj.mexhosts(veh_indices(i_veh)) = mexhost;
                    feval(obj.mexhosts(veh_indices(i_veh)), 'optimizer', Function.InitializeWithScenario, obj.scenario);
                end

                % if centralized only 1 mexhost is neccessary
            else
                optimizer(Function.InitializeWithScenario, obj.scenario);
            end

        end

    end

    methods (Access = protected)

        function tree = create_tree(obj, iter)
            trim = iter.trim_indices;
            x = iter.x0(:, 1);
            y = iter.x0(:, 2);
            yaw = iter.x0(:, 3);
            k = 0;
            g = 0;
            h = 0;
            tree = Tree(x, y, yaw, trim, k, g, h);
        end

    end

end
