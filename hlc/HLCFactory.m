classdef HLCFactory < handle

    properties (Access = public)
    end

    methods

        function obj = HLCFactory()
        end

        function hlc = get_hlc(obj, options, plant, vehicle_ids, do_dry_run)
            arguments
                obj
                options
                plant
                vehicle_ids
                do_dry_run = true;
            end

            if false % do_dry_run
                % FIXME does not work in parallel
                obj.dry_run_hlc(options, plant.controlled_vehicle_ids);
            end

            if options.is_prioritized

                if length(vehicle_ids) == 1
                    % PB Controller for exactly 1 vehicle. Communicates
                    % with the other HLCs
                    hlc = PrioritizedParallelController(options, plant);
                else
                    % PB Controller controlling all vehicles
                    hlc = PrioritizedSequentialController(options, plant);
                end

            else
                hlc = CentralizedController(options, plant);
            end

        end

    end

    methods (Access = private)

        % This function runs the HLC once without outputting anything and
        % resets it afterwards.
        % Usually MATLAB takes some time to run code for the first time because
        % it has to compile it while running. If we run if before the
        % experiment starts, we save a few hundred milliseconds on the first
        % actual time step of the experiment.
        % This so far only works when the HLC doesn't require input arguments
        % It also assumes that if the HLC is distributed, all other HLCs start
        % at approximately the same time, because otherwise onEachTimestep
        % won't run through.
        %
        % Important note: This might take some time depending on how hard to
        % solve the first time step of this scenario is.

        function dry_run_hlc(obj, options, dry_run_vehicle_ids)
            fprintf("Dry run of HLC...");

            % use simulation to avoid communication with a lab
            options.environment = Environment.Simulation;
            % for simulation manual vehicles are disabled
            options.manual_control_config.is_active = false;
            % for the dry run plotting is not necessary
            options.options_plot_online.is_active = false;
            % for the dry run reduce experiment time to a minimum
            options.T_end = 2 * options.dt_seconds;
            % for the dry run results are not relevant
            options.should_save_result = false;

            plant = Plant.get_plant(options.environment);
            plant.setup(options, dry_run_vehicle_ids);

            hlc = obj.get_hlc(options, plant, dry_run_vehicle_ids, false);
            hlc.run();

            fprintf(" done.\n");
        end

    end

end
