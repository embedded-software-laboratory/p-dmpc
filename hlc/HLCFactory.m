classdef HLCFactory < handle

    properties (Access = public)
    end

    methods

        function obj = HLCFactory()
        end

        % Optional argument wether to do a dry run of the first timestep beforehand
        % dry_run can massively decrease the time needed for the first
        % timestep during the experiment.
        function hlc = get_hlc(obj, scenario, plant, vehicle_ids, dry_run)

            if dry_run
                obj.dry_run_hlc(scenario, plant.all_vehicle_ids);
            end

            if scenario.options.is_prioritized

                if length(vehicle_ids) == 1
                    % PB Controller for exactly 1 vehicle. Communicates
                    % with the other HLCs
                    hlc = PrioritizedParallelController(scenario, plant);
                else
                    % PB Controller controlling all vehicles
                    hlc = PrioritizedSequentialController(scenario, plant);
                end

            else
                hlc = CentralizedController(scenario, plant);
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

        function dry_run_hlc(obj, scenario, dry_run_vehicle_ids)
            disp("Starting dry run of HLC");

            % use simulation to avoid communication with a lab
            scenario.options.environment = Environment.Simulation;
            % for simulation manual vehicles are disabled
            scenario.options.manual_control_config.is_active = false;
            % for the dry run plotting is not necessary
            scenario.options.options_plot_online.is_active = false;
            % for the dry run reduce experiment time to a minimum
            scenario.options.T_end = 2 * scenario.options.dt_seconds;
            % for the dry run results are not relevant
            scenario.options.should_save_result = false;

            plant = PlantFactory.get_experiment_interface(scenario.options.environment);
            plant.setup(scenario.options, scenario, dry_run_vehicle_ids);

            hlc = obj.get_hlc(scenario, plant, dry_run_vehicle_ids, false);
            hlc.run();

            disp("Dry Run Completed");
        end

    end

end
