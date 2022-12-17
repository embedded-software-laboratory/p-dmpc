classdef HLCFactory < handle
    properties (Access=public)
        % scenario variable
        scenario

        % data queue for visualization. Used if running distributed HLCs
        % locally with Parallel Computing Toolbox
        visualization_data_queue
    end

    methods
        % Set default settings
        function obj = HLCFactory()
            % Some default values are invalid and thus they're easily spotted when they haven't been explicitly set
            % We can then either throw an exception or use an arbitrary option when we find a default value
            % Or should we make valid and useful default values?
            obj.scenario = [];
            obj.visualization_data_queue = [];
        end

        % Optional argument wether to do a dry run of the first timestep beforehand
        % dry_run can massively decrease the time needed for the first
        % timestep during the experiment.
        function hlc = get_hlc( obj, vehicle_ids, dry_run )

            if isempty(obj.scenario)
                throw(MException('HlcFactory:InvalidState', 'HlcScenario not set'));
            end

            % If the user doesn't specify otherwise, we do a dry run beforehand
            if nargin < 3
                dry_run = true;
            end
            if dry_run
                obj.dryRunHlc();
            end

            if obj.scenario.options.scenario_name == Scenario_Type.Commonroad
                if obj.scenario.options.isPB
                    hlc = PbControllerParl();
                    controller_name = strcat('par. PB-', hlc.controller_name, ' ', char(obj.scenario.options.priority));
                else
                    hlc = CentralizedController();
                    controller_name = strcat('centralized-', hlc.controller_name, ' ', char(obj.scenario.options.priority));
                end
            else
                if obj.scenario.options.isPB
                    hlc = PbControllerParl();
                    controller_name = strcat(hlc.controller_name, '-PB');
                else
                    hlc = CentralizedController();
                    controller_name = strcat(hlc.controller_name, '-centralized');
                end
            end

            %TODO filter scenario for veh id
            hlc.set_scenario(obj.scenario);

            hlc.set_controller_name(controller_name);

            hlc.set_vehicle_ids(vehicle_ids);

            hlc.set_hlc_adapter(obj.visualization_data_queue);

        end

        function set_scenario( obj, scenario )
            obj.scenario = scenario;
        end

        function set_visualization_data_queue( obj )
            obj.visualization_data_queue = parallel.pool.DataQueue;
        end
    end

    methods (Access=private)

        % This function runs the HLC once without outputting anything and
        % resets it afterwards.
        % Usually MATLAB takes some time to run code for the first time because
        % it has to compile it while running. If we run if before the
        % experiment starts, we save a few hundred milliseconds on the first
        % actual timestep of the experiment.
        % This so far only works when the HLC doesn't require input arguments
        % It also assumes that if the HLC is distributed, all other HLCs start
        % at approximately the same time, because otherwise onEachTimestep
        % won't run through.
        %
        % Important note: This might take some time depending on how hard to
        % solve the first timestep of this scenario is.
        function dryRunHlc(obj)
            disp("Starting dry run of HLC - TODO Implement");

            % Reset visualization + result + iter etc. to its initial value
            % TODO implement
        end
    end

end
