function [result, scenario] = main_distributed(vehicle_id)

    %startup();

    if verLessThan('matlab', '9.12')
        warning("Code is developed in MATLAB 2022a, prepare for backward incompatibilities.")
    end

    % read config from disk
    options = Config();
    options = options.importFromJson(fileread('Config.json'));

    % read scenario from disk
    scenario = load('scenario.mat', 'scenario').scenario;

    dry_run = (options.environment == Environment.CpmLab || options.environment == Environment.SimulationDistributed); % TODO: Use dry run also for unified lab api?
    plant = PlantFactory.get_experiment_interface(options.environment);
    % set active vehicle IDs and initialize communication
    plant.setup(options, scenario, options.path_ids, vehicle_id);

    % In priority-based computation, vehicles communicate via ROS 2.
    % main.m will have deleted the ros2 message types before distributing the code.
    % Therefore, the the ros2 message types need to be created here before experiment setup.
    generate_ros2_msgs();

    % get HLC
    factory = HLCFactory();
    factory.set_scenario(scenario);

    if options.is_prioritized == true
        hlc = factory.get_hlc(scenario, vehicle_id, dry_run, plant);
        [result, scenario] = hlc.run();
    else
        warning("Use main_distributed.m only for pb-scenarios.")
    end

end
