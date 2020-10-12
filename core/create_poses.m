function [init_poses, target_poses] = create_poses(scenario)
%CREATE_POSES Read out a scenario to create poses
    target_poses = [];
    init_poses = [];

    n_veh = length(scenario.vehicles);
    for i = 1:n_veh
        init_pose.x = scenario.vehicles(i).referenceTrajectory(1,1);
        init_pose.y = scenario.vehicles(i).referenceTrajectory(1,2);
        init_pose.yaw = scenario.vehicles(i).heading;
        target_pose.x = scenario.vehicles(i).referenceTrajectory(2,1);
        target_pose.y = scenario.vehicles(i).referenceTrajectory(2,2);
        target_pose.yaw = 0;
        init_poses = [init_poses, init_pose];
        target_poses = [target_poses, target_pose];
    end
end

