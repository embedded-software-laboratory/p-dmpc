classdef HLCFactory < handle

    properties (Access = public)
    end

    % TODO all functions could be static
    methods

        function obj = HLCFactory()
        end

        function hlc = get_hlc(obj, options, controlled_vehicles, optional)

            arguments
                obj
                options
                % controlled_vehicles Depending on the Plant, these can be
                % Simulation:   indices of the vehicles
                % CpmLab:       IDs of the vehicles
                controlled_vehicles
                optional.do_dry_run = false
                optional.ros2_node = []
            end

            if isempty(optional.ros2_node)
                % Create ROS2 node for this HLC
                vehicle_ids_string = sprintf('_%02d', controlled_vehicles);
                optional.ros2_node = ros2node(['hlc', vehicle_ids_string]);
            end

            plant = Plant.get_plant(options.environment, optional.ros2_node);
            plant.setup(options, controlled_vehicles);

            if optional.do_dry_run
                % FIXME does not work in parallel
                obj.dry_run_hlc(options, plant.vehicle_indices_controlled, optional.ros2_node);
            end

            if options.is_prioritized

                if length(controlled_vehicles) == 1
                    % Prioritized controller for exactly 1 vehicle. Communicates
                    % with the other HLCs
                    hlc = PrioritizedController(options, plant, optional.ros2_node);
                else
                    % Prioritized controller controlling all vehicles
                    hlc = PrioritizedSequentialController();

                    for i_vehicle = controlled_vehicles
                        sub_hlc = obj.get_hlc( ...
                            options, ...
                            i_vehicle, ...
                            ros2_node = optional.ros2_node ...
                        );
                        hlc.add_hlc(sub_hlc);
                    end

                end

            else
                % Centralized Controller controlling all vehicles
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

        function dry_run_hlc(obj, options, dry_run_vehicle_ids, ros2_node)
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

            plant = Plant.get_plant(options.environment, ros2_node);
            plant.setup(options, dry_run_vehicle_ids);

            hlc = obj.get_hlc(options, dry_run_vehicle_ids, do_dry_run = true);
            hlc.run();

            fprintf(" done.\n");
        end

    end

end
