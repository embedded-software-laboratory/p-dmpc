function [result, scenario] = main_distributed(vehicle_id)

    %startup();

    if verLessThan('matlab', '9.12')
        warning("Code is developed in MATLAB 2022a, prepare for backward incompatibilities.")
    end

    % read scenario from disk
    scenario = load('scenario.mat', 'scenario').scenario;

    dry_run = (scenario.options.environment == Environment.CpmLab || scenario.options.environment == Environment.SimulationDistributed); % TODO: Use dry run also for unified lab api?
    plant = PlantFactory.get_experiment_interface(scenario.options.environment);
    % set active vehicle IDs and initialize communication
    plant.setup(scenario, [], vehicle_id);

    % In priority-based computation, vehicles communicate via ROS 2.
    % main.m will have deleted the ros2 message types before distributing the code.
    % Therefore, the the ros2 message types need to be created here before experiment setup.
    generate_ros2_msgs();

    % get HLC
    factory = HLCFactory();
    factory.set_scenario(scenario);

    if scenario.options.is_prioritized == true
        hlc = factory.get_hlc(vehicle_id, dry_run, plant);
        [result, scenario] = hlc.run();
    else
        warning("Use main_distributed.m only for pb-scenarios.")
    end

end
