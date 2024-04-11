classdef HlcFactory

    properties (Access = public)
    end

    methods (Static, Access = public)

        function hlc = get_hlc(options, vehicle_indices_controlled, optional)

            arguments
                options Config
                vehicle_indices_controlled (1, :) double
                optional.do_dry_run = false
                optional.ros2_node = []
            end

            % Dry run must be executed before everything else
            if optional.do_dry_run
                HlcFactory.dry_run_hlc(options, vehicle_indices_controlled);
            end

            if isempty(optional.ros2_node)
                % Create ROS2 node for this HLC
                vehicle_ids_string = sprintf('_%02d', vehicle_indices_controlled);
                optional.ros2_node = ros2node(['hlc', vehicle_ids_string]);
            end

            plant = Plant.get_plant(options.environment, optional.ros2_node);
            plant.setup(options, vehicle_indices_controlled);

            if options.is_prioritized

                if length(vehicle_indices_controlled) == 1
                    % Prioritized controller for exactly 1 vehicle. Communicates
                    % with the other HLCs
                    if options.priority == PriorityStrategies.optimal_priority
                        hlc = PrioritizedOptimalController(options, plant, optional.ros2_node);
                    elseif options.priority == PriorityStrategies.explorative_priority
                        hlc = PrioritizedExplorativeController(options, plant, optional.ros2_node);
                    else
                        hlc = PrioritizedController(options, plant, optional.ros2_node);
                    end

                else

                    % Prioritized controller controlling all vehicles
                    if options.priority == PriorityStrategies.optimal_priority
                        hlc = PrioritizedOptimalSequentialController();
                    elseif options.priority == PriorityStrategies.explorative_priority
                        hlc = PrioritizedExplorativeSequentialController();
                    else
                        hlc = PrioritizedSequentialController();
                    end

                    for i_vehicle = vehicle_indices_controlled
                        sub_hlc = HlcFactory.get_hlc( ...
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

    methods (Static, Access = private)

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

        function dry_run_hlc(options, dry_run_vehicle_indices)

            arguments
                options Config
                dry_run_vehicle_indices (1, :) double
            end

            fprintf("Dry run of HLC...\n");

            % dry_run must be false otherwise it would lead to an endless loop
            % (get_hlc -> dry_run_hlc -> get_hlc)
            hlc = HlcFactory.get_hlc(options, dry_run_vehicle_indices, do_dry_run = false);
            hlc.set_clean_up_dry_run_function();
            hlc.run();

            fprintf("Dry run done.\n");
        end

    end

end
