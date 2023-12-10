function [experiment_result, scenario] = main_distributed(vehicle_id)

    %startup();

    if isMATLABReleaseOlderThan('R2023a')
        warning("Code is developed in MATLAB R2023a, prepare for backward incompatibilities.")
    end

    % read config from disk
    options = Config.load_from_file('Config.json');

    % read scenario from disk
    scenario = load('scenario.mat', 'scenario').scenario;

    plant = Plant.get_plant(options.environment);
    % set active vehicle IDs and initialize communication
    plant.setup(options, options.path_ids, vehicle_id);

    % In prioritized computation, vehicles communicate via ROS 2.
    % main.m will have deleted the ros2 message types before distributing the code.
    % Therefore, the the ros2 message types need to be created here before experiment setup.
    generate_ros2_msgs();

    % get HLC
    factory = HLCFactory();

    if options.is_prioritized == true
        hlc = factory.get_hlc(options, plant, vehicle_id, options.is_dry_run);
        experiment_result = hlc.run();
    else
        warning("Use main_distributed.m only for pb-scenarios.")
    end

end
