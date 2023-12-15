function [experiment_result, scenario] = main_distributed(vehicle_id)

    %startup();

    if isMATLABReleaseOlderThan('R2023a')
        warning("Code is developed in MATLAB R2023a, prepare for backward incompatibilities.")
    end

    % read config from disk
    options = Config.load_from_file('Config.json');

    % read scenario from disk
    scenario = load('scenario.mat', 'scenario').scenario;

    % In prioritized computation, vehicles communicate via ROS 2.
    % main.m will have deleted the ros2 message types before distributing the code.
    % Therefore, the the ros2 message types need to be created here before experiment setup.
    generate_ros2_msgs();

    % get HLC
    factory = HLCFactory();

    if options.is_prioritized
        hlc = factory.get_hlc(options, vehicle_id);
        experiment_result = hlc.run();
    else
        warning("Use main_distributed.m only for prioritized scenarios.")
    end

end
