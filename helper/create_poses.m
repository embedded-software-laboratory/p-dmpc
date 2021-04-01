function [init_poses, target_poses] = create_poses(scenario)
    n_veh = length(scenario.vehicles);
    init_poses.xs = zeros(1, n_veh);
    init_poses.ys = zeros(1, n_veh);
    init_poses.yaws = zeros(1, n_veh);
    target_poses.xs = zeros(1, n_veh);
    target_poses.ys = zeros(1, n_veh);
    target_poses.yaws = zeros(1, n_veh);
    for i = 1:n_veh
        init_poses.xs(i) = scenario.vehicles(i).referenceTrajectory(1,1);
        init_poses.ys(i) = scenario.vehicles(i).referenceTrajectory(1,2);
        init_poses.yaws(i) = scenario.vehicles(i).yaw;
        target_poses.xs(i) = scenario.vehicles(i).referenceTrajectory(2,1);
        target_poses.ys(i) = scenario.vehicles(i).referenceTrajectory(2,2);
        target_poses.yaws(i) = scenario.vehicles(i).yaw;
    end
end

