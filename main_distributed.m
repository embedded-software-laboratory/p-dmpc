function experiment_result = main_distributed(i_vehicle, optional)

    arguments
        i_vehicle (1, 1) double
        optional.options (1, 1) Config = Config.load_from_file('Config.json')
    end

    % In prioritized computation, vehicles communicate via ROS 2.
    % main.m will have deleted the ros2 message types before distributing the code.
    % Therefore, the the ros2 message types need to be created here before experiment setup.
    generate_ros2_msgs();

    hlc = HlcFactory.get_hlc( ...
        optional.options, ...
        i_vehicle, ...
        do_dry_run = optional.options.should_do_dry_run ...
    );
    experiment_result = hlc.run();

end
