function result = main_lab(varargin)
%MAIN_MPC Summary of this function goes here
%   Detailed explanation goes here
%% Setup
% Add modules to path
addpath(genpath(pwd));

close all
clear -regex ^(?!varargin$).*$
clc

%% Read input
vehicle_ids = [varargin{1:end-1}];
assert(issorted(vehicle_ids));

%% Initialize data readers/writers...
% getenv('HOME'), 'dev/software/high_level_controller/examples/matlab' ...
common_cpm_functions_path = fullfile( ...
    '../examples/matlab' ...
);
assert(isfolder(common_cpm_functions_path), 'Missing folder "%s".', common_cpm_functions_path);
addpath(common_cpm_functions_path);

matlabDomainId = 1;
[matlabParticipant, reader_vehicleStateList, writer_vehicleCommandTrajectory, ~, reader_systemTrigger, writer_readyStatus, trigger_stop] = init_script(matlabDomainId);

% Set reader properties
reader_vehicleStateList.WaitSet = true;
reader_vehicleStateList.WaitSetTimeout = 5; % [s]

%% Setup the HLC
% Add modules to path
addpath(genpath(pwd));
% warning('off','MATLAB:polyshape:repairedBySimplify')

close all
clear -regex ^(?!varargin$).*$
clc

% Determine options
options = selection(varargin{end});

% Setup scenario
scenario = Scenario(options);

% Setup controller
info = struct;
info.trim_indices = [scenario.vehicles(:).trim_config];
% Initialize
n_vertices = 0;
idx = tree.nodeCols();
cur_depth = 1;

controller = @(scenario, iter)...
    graph_search(scenario, iter);


% init result struct
result = get_result_struct(scenario,50);
controller_init = false;
% times
t             = zeros(1,0);
t_start_nanos = 0;
t_end         = 10;
t_rel         = 0;

% Middleware period for valid_after stamp
dt_period_nanos = uint64(scenario.dt*1e9);


%% Sync start with infrastructure
% Send ready signal for all assigned vehicle ids
disp('Sending ready signal');
for iVehicle = vehicle_ids
    ready_msg = ReadyStatus;
    ready_msg.source_id = strcat('hlc_', num2str(iVehicle));
    ready_stamp = TimeStamp;
    ready_stamp.nanoseconds = uint64(0);
    ready_msg.next_start_stamp = ready_stamp;
    writer_readyStatus.write(ready_msg);
end

% Wait for start or stop signal
disp('Waiting for start or stop signal');    
got_stop = false;
got_start = false;
while (~got_stop && ~got_start)
    [got_start, got_stop] = read_system_trigger(reader_systemTrigger, trigger_stop);
end

%% Main control loop
while (~got_stop)
    % Measurement
    % -------------------------------------------------------------------------
    [sample, ~, sample_count, ~] = reader_vehicleStateList.take();
    if (sample_count > 1)
        warning('Received %d samples, expected 1. Correct middleware period? Missed deadline?', sample_count);
    end
    
    
    % Control 
    % -------------------------------------------------------------------------
    % one-time initialization of starting position
    if controller_init == false
        t_start_nanos = sample(end).t_now;
        x0 = zeros(scenario.nVeh,4);
        pose = [sample(end).state_list.pose];
        x0(:,1) = [pose.x];
        x0(:,2) = [pose.y];
        x0(:,3) = [pose.yaw];
        x0(:,4) = [sample(end).state_list.speed];
        controller_init = true;
    else
        % take last planned state as new actual state
        planned_node = info.tree.Node{info.tree_path(2)};
        speeds = zeros(scenario.nVeh,1);
        for iVeh=1:scenario.nVeh
            speeds(iVeh) = scenario.mpa.trims(planned_node(iVeh,idx.trim)).speed;
        end
        x0 = [planned_node(:,idx.x), planned_node(:,idx.y), planned_node(:,idx.yaw), speeds];
    end
    % Sample reference trajectory
    iter = rhc_init(scenario,x0,info.trim_indices);
    result.iteration_structs{cur_depth} = iter;
    controller_timer = tic;
        [u, y_pred, info] = controller(scenario, iter);
    result.controller_runtime(cur_depth) = toc(controller_timer);
    % save controller outputs in resultstruct
    result.trajectory_predictions(:,cur_depth) = y_pred;
    result.controller_outputs{cur_depth} = u;
            
    n_vertices = n_vertices + length(info.tree.Node);
            
    % Apply control action f/e veh
    % -------------------------------------------------------------------------
    out_of_map_limits = false(scenario.nVeh,1);
    for iVeh = 1:scenario.nVeh
        n_traj_pts = numel(info.tree_path)-1;
        n_predicted_points = size(y_pred{iVeh},1);
        idx_predicted_points = 1:n_predicted_points/n_traj_pts:n_predicted_points;
        trajectory_points(1:n_traj_pts) = TrajectoryPoint;
        for i_traj_pt = 1:n_traj_pts
            i_predicted_points = idx_predicted_points(i_traj_pt);
            trajectory_points(i_traj_pt).t.nanoseconds = ...
                uint64(sample(end).t_now + i_traj_pt*dt_period_nanos);
            trajectory_points(i_traj_pt).px = y_pred{iVeh}(i_predicted_points,1);
            trajectory_points(i_traj_pt).py = y_pred{iVeh}(i_predicted_points,2);
            yaw = y_pred{iVeh}(i_predicted_points,3);
            speed = scenario.mpa.trims(y_pred{iVeh}(i_predicted_points,4)).speed;
            trajectory_points(i_traj_pt).vx = cos(yaw)*speed;
            trajectory_points(i_traj_pt).vy = sin(yaw)*speed;
        end
        out_of_map_limits(iVeh) = is_veh_at_map_border(trajectory_points);
        vehicle_command_trajectory = VehicleCommandTrajectory;
        vehicle_command_trajectory.vehicle_id = uint8(vehicle_ids(iVeh));
        vehicle_command_trajectory.trajectory_points = trajectory_points;
        vehicle_command_trajectory.header.create_stamp.nanoseconds = ...
            uint64(sample(end).t_now);
        vehicle_command_trajectory.header.valid_after_stamp.nanoseconds = ...
            uint64(sample(end).t_now + dt_period_nanos);
        writer_vehicleCommandTrajectory.write(vehicle_command_trajectory);
    end

    % Check for stop signal
    % -------------------------------------------------------------------------
    [~, got_stop] = read_system_trigger(reader_systemTrigger, trigger_stop);
    if any(out_of_map_limits)
        got_stop = true;
    end


    cur_depth = cur_depth + 1;
end

end


function stop_experiment = is_veh_at_map_border(trajectory_points)
    % Vehicle command timeout is 1000 ms after the last valid_after_stamp,
    % so vehicle initiates stop between third and fourth trajectory point
    x_min = 0;
    x_max = 4.5;
    y_min = 0;
    y_max = 4.0;
    px = trajectory_points(5).px;
    py = trajectory_points(5).py;
    stop_experiment = x_min>px || px>x_max ...
                   || y_min>py || py>y_max;
end