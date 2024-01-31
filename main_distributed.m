function experiment_result = main_distributed(i_vehicle)

    % read config from disk
    options = Config.load_from_file('Config.json');

    % In prioritized computation, vehicles communicate via ROS 2.
    % main.m will have deleted the ros2 message types before distributing the code.
    % Therefore, the the ros2 message types need to be created here before experiment setup.
    generate_ros2_msgs();

    hlc = HlcFactory.get_hlc(options, i_vehicle);
    experiment_result = hlc.run();

end
