function create_publishers(hlc)
    % create_publishers This function creates all ROS publishers for the given hlc.
    % ROS 2 nodes are created for each vehicle. Each vehicle has its own topic
    % and sends its data only to its own topic.
    %
    % INPUT:
    %   hlc: Controller implementing the HighLevelController
    %

    % generate custom message type (for vehicle communication) if not exist
    msgList = ros2("msg", "list"); % get all ROS 2 message types
    [file_path, ~, ~] = fileparts(mfilename('fullpath'));

    if ((sum(cellfun(@(c)strcmp(c, 'veh_msgs/Traffic'), msgList)) == 0) || (sum(cellfun(@(c)strcmp(c, 'veh_msgs/Predictions'), msgList)) == 0))
        % if the message type 'veh_msgs/Traffic' does not exist
        path_custom_msg = [file_path, filesep, 'cust1'];

        % Generate custom messages. NOTE that Python, CMake, and a C++ compiler are required (see
        % https://de.mathworks.com/help/ros/gs/ros-system-requirements.html
        % for more details according to your own MATLAB version).
        %
        % Useful functions:
        % 1. pyenv % check python version used by MATLAB
        % 2. pyenv('Version','requiredPythonVersionNumber') or pyenv('Version','fullPathOfPythonExe')
        % 3. !cmake --version % CMake version
        % 4. mex -setup % set c language compiler
        %
        % Note that sometimes ros2genmsg fails although all denpendencies
        % exist because the path where the custom messages are stored is
        % too deep. Try to move them to shallower path and try again.
        disp('Generating ROS 2 custom message type...')

        try
            ros2genmsg(path_custom_msg)
        catch ME
            disp(['If all environments for ros2genmsg() are prepared but still failed, try to move the whole folder to a ' ...
                  'shallower path and run again if you use Windows machine, which sadly has a max path limit constraint.'])
            throw(ME)
        end

    end

    if sum(cellfun(@(c)strcmp(c, 'ros_g29_force_feedback/ForceFeedback'), msgList)) == 0

        if hlc.scenario.options.environment == Environment.CpmLab
            % This message type is only needed for lab mode but not
            % simulation mode
            path_custom_msg = [file_path, filesep, '../../manual_control'];
            disp('Generating ROS 2 custom message type...')
            ros2genmsg(path_custom_msg)
        end

    end

    %     topicList = ros2("topic","list");
    %     nodeList = ros2("node","list");

    timer_nodes = tic;
    disp('Creating ROS 2 nodes...');

    for vehicle_id = hlc.plant.controlled_vehicle_ids
        index = find(vehicle_id == hlc.plant.all_vehicle_ids);
        % create instance of the communication class
        hlc.traffic_communication{index} = TrafficCommunication();
        % create node and store topic name and message type
        hlc.traffic_communication{index}.initialize(vehicle_id, ...
            'traffic', '/vehicle_traffic', 'veh_msgs/Traffic');

        % create instance of the communication class
        hlc.predictions_communication{index} = PredictionsCommunication();
        % create node and store topic name and message type
        hlc.predictions_communication{index}.initialize(vehicle_id, ...
            'prediction', '/vehicle_prediction', 'veh_msgs/Predictions');
    end

    duration_nodes = toc(timer_nodes);
    disp(['Finished creating nodes in ', ...
              num2str(duration_nodes), ' seconds.']);

    timer_publisher = tic;
    disp('Creating ROS 2 publishers...');

    for vehicle_index = hlc.plant.indices_in_vehicle_list
        % create publishers
        hlc.traffic_communication{vehicle_index}.create_publisher();
        hlc.predictions_communication{vehicle_index}.create_publisher();
    end

    duration_publisher = toc(timer_publisher);
    disp(['Finished creating publishers in ', ...
              num2str(duration_publisher), ' seconds.']);

end
