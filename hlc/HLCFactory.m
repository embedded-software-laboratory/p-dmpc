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
                obj.dry_run_hlc(vehicle_ids);
            end

            if obj.scenario.options.isPB
                if length(vehicle_ids)==1
                    % PB Controller for exactly 1 vehicle. Communicates
                    % with the other HLCs
                    hlc = PbControllerParl();
                else
                    % PB Controller controlling all vehicles
                    hlc = PbControllerSeq();
                end
            else
                hlc = CentralizedController();
            end

            %TODO filter scenario for veh id
            hlc.set_scenario(obj.scenario);

            hlc.set_controller_name(obj.get_controller_name(obj.scenario.options));

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

    methods (Static)
        function controller_name = get_controller_name(options)
            if options.isPB
                if options.isParl
                    controller_name = strcat('par. PB-','RHGS-', char(options.priority));
                else
                    controller_name = strcat('seq. PB-','RHGS-', char(options.priority));
                end
            else
                controller_name = strcat('centralized-', 'RHGS-', char(options.priority));
            end
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
        function dry_run_hlc(obj, vehicle_ids)
            disp("Starting dry run of HLC");
            visu_backup = obj.scenario.options.visu;
            is_sim_lab_backup = obj.scenario.options.is_sim_lab;
            T_end_backup = obj.scenario.options.T_end;
            save_result_backup = obj.scenario.options.isSaveResult;
            % avoid sending any data to Cpm Lab. Thus, use Sim Lab
            obj.scenario.options.is_sim_lab = true;
            obj.scenario.options.visu = [false, false];
            obj.scenario.options.T_end = 2 * obj.scenario.options.dt;
            obj.scenario.options.isSaveResult = false;
            hlc = obj.get_hlc(vehicle_ids, false);
            hlc.run();
            obj.scenario.options.is_sim_lab = is_sim_lab_backup;
            obj.scenario.options.visu = visu_backup;
            obj.scenario.options.T_end = T_end_backup;
            obj.scenario.options.isSaveResult = save_result_backup;
            disp("Dry Run Completed");
        end
    end
end