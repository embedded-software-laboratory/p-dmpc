classdef HLCFactory < handle

    properties (Access = public)
        % scenario variable
        scenario

    end

    methods
        % Set default settings
        function obj = HLCFactory()
            % Some default values are invalid and thus they're easily spotted when they haven't been explicitly set
            % We can then either throw an exception or use an arbitrary option when we find a default value
            % Or should we make valid and useful default values?
            obj.scenario = [];

        end

        % Optional argument wether to do a dry run of the first timestep beforehand
        % dry_run can massively decrease the time needed for the first
        % timestep during the experiment.
        function hlc = get_hlc(obj, vehicle_ids, dry_run, plant)

            if isempty(obj.scenario)
                throw(MException('HlcFactory:InvalidState', 'HlcScenario not set'));
            end

            % If the user doesn't specify otherwise, we do a dry run beforehand
            if nargin < 3
                dry_run = true;
            end

            if dry_run
                obj.dry_run_hlc(plant.all_vehicle_ids);
            end

            if obj.scenario.options.is_prioritized

                if length(vehicle_ids) == 1
                    % PB Controller for exactly 1 vehicle. Communicates
                    % with the other HLCs
                    hlc = PrioritizedParallelController(obj.scenario, plant);
                else
                    % PB Controller controlling all vehicles
                    hlc = PrioritizedSequentialController(obj.scenario, plant);
                end

            else
                hlc = CentralizedController(obj.scenario, plant);
            end

            hlc.set_controller_name(obj.get_controller_name(obj.scenario.options));

        end

        function set_scenario(obj, scenario)
            obj.scenario = scenario;
        end

    end

    methods (Static)

        function controller_name = get_controller_name(options)

            if options.is_prioritized

                if options.compute_in_parallel
                    controller_name = strcat('par. PB-', 'RHGS-', char(options.priority));
                else
                    controller_name = strcat('seq. PB-', 'RHGS-', char(options.priority));
                end

            else
                controller_name = strcat('centralized-', 'RHGS-', char(options.priority));
            end

        end

    end

    methods (Access = private)

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
        function dry_run_hlc(obj, dry_run_vehicle_ids)
            disp("Starting dry run of HLC");
            plot_backup = obj.scenario.options.options_plot_online.is_active;
            environment_backup = obj.scenario.options.environment;
            T_end_backup = obj.scenario.options.T_end;
            save_result_backup = obj.scenario.options.should_save_result;
            % avoid sending any data to Cpm Lab. Thus, use Sim Lab
            obj.scenario.options.environment = Environment.Simulation;
            plant = PlantFactory.get_experiment_interface(obj.scenario.options.environment);
            obj.scenario.options.options_plot_online.is_active = false;
            obj.scenario.options.T_end = 2 * obj.scenario.options.dt_seconds;
            obj.scenario.options.should_save_result = false;
            plant.setup(obj.scenario, dry_run_vehicle_ids);

            hlc = obj.get_hlc(dry_run_vehicle_ids, false, plant);
            hlc.run();
            obj.scenario.options.environment = environment_backup;
            obj.scenario.options.options_plot_online.is_active = plot_backup;
            obj.scenario.options.T_end = T_end_backup;
            obj.scenario.options.should_save_result = save_result_backup;

            if obj.scenario.options.use_cpp()

                if ismac()
                    % clear mex dont work on ARM Mac
                    [~, result] = system('sysctl machdep.cpu.brand_string');
                    matches = regexp(result, 'machdep.cpu.brand_string: Apple M[1-9]( Pro| Max)?', 'match');

                    if isempty(matches)
                        clear mex;
                    end

                else
                    clear mex;
                end

            end

            disp("Dry Run Completed");
        end

    end

end
