classdef IterationData
%ITERATIONDATA  Class as data structure for data that changes in every iteration during the experiment.

    properties
        referenceTrajectoryPoints
        referenceTrajectoryIndex
        k                                               % current time/simulation step
        x0                                              % state
        trim_indices                                    % current trim
        v_ref                                           % reference speed  
        predicted_lanelets                              % vehicle's predicted lanelets
        predicted_lanelet_boundary                      % first column for left boundary, second column for right boundary, third column for MATLAB polyshape instance
        reachable_sets                                  % cells to store instances of MATLAB calss `polyshape`
        occupied_areas                                  % currently occupied areas with normal offset of vehicles 
        emergency_maneuvers                             % occupied area of emergency braking maneuver
        last_trajectory_index                           % initial trajectory index
        lane_change_lanes                               % positions of the lane before the lane change and the lane change lane in the lanelet vector
        lane_change_indices                             % indices of the trajectory points of the lane before the lane change and the lane change lane
        lanes_before_update                             % lanes before path has been automatically updated in CPM Lab mode
        auto_updated_path                               % set in rhc_init to memorize when path is updated automatically
        adjacency;                                      % (nVeh x nVeh) matrix, entry is 1 if two vehicles drive in two adjacent lanelets and their distance are smaller enough
        semi_adjacency;                                 % OUTDATED (nVeh x nVeh) matrix, entry is 1 if two vehicles drive in two semi-adjacent lanelets and their distance are smaller enough
    end

    methods
        function obj = IterationData(scenario,k)
            nVeh = scenario.options.amount;
            Hp = scenario.options.Hp;
            obj.k = k;
            obj.referenceTrajectoryPoints = zeros(nVeh,Hp,2);
            obj.referenceTrajectoryIndex = zeros(nVeh,Hp,1);
            obj.x0 = zeros(nVeh, 4);
            obj.trim_indices = zeros(nVeh, 1);
            obj.v_ref = zeros(nVeh,Hp);
            obj.predicted_lanelets = cell(nVeh, 1);
            obj.predicted_lanelet_boundary = cell(nVeh, 3);
            obj.reachable_sets = cell(nVeh, Hp);
            obj.occupied_areas = cell(nVeh, 1);
            obj.emergency_maneuvers = cell(nVeh, 1);
            obj.last_trajectory_index = ones(nVeh,1)*10;
            obj.lane_change_lanes = zeros(nVeh,10,2);
            obj.lane_change_indices = zeros(nVeh,10,4);
            obj.lanes_before_update = zeros(nVeh,1,2);
            obj.auto_updated_path = false(nVeh,1);
            obj.adjacency = zeros(nVeh,nVeh);
            obj.semi_adjacency = zeros(nVeh,nVeh);
        end
    end


end