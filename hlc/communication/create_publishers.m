function create_publishers(hlc)
    % creates ros2 publisher for the given high level controller
    % for each controlled vehicle a ros2 node is created
    % all vehicles share topics for related messages
    %
    % INPUT:
    %   hlc: handle of high level controller
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

    timer_nodes = tic;
    disp('Creating ROS 2 nodes...');

    for vehicle_index = hlc.plant.indices_in_vehicle_list
        vehicle_id = hlc.plant.all_vehicle_ids(vehicle_index);
        % create instance of the communication class
        hlc.traffic_communication{vehicle_index} = TrafficCommunication();
        % create node and store topic name and message type
        hlc.traffic_communication{vehicle_index}.initialize( ...
            vehicle_id, ...
            'traffic', ...
            '/vehicle_traffic', ...
            'veh_msgs/Traffic' ...
        );

        % create instance of the communication class
        hlc.predictions_communication{vehicle_index} = PredictionsCommunication();
        % create node and store topic name and message type
        hlc.predictions_communication{vehicle_index}.initialize( ...
            vehicle_id, ...
            'prediction', ...
            '/vehicle_prediction', ...
            'veh_msgs/Predictions' ...
        );
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
