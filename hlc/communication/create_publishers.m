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
            path_custom_msg = [file_path, filesep, 'cust2'];
            disp('Generating ROS 2 custom message type...')
            ros2genmsg(path_custom_msg)
        end

    end

    %     topicList = ros2("topic","list");
    %     nodeList = ros2("node","list");
    if isempty(hlc.scenario.vehicles(hlc.plant.indices_in_vehicle_list(1)).communicate)
        start = tic;
        disp('Creating ROS 2 publishers...')

        for index = hlc.plant.indices_in_vehicle_list
            hlc.scenario.vehicles(index).communicate.traffic = TrafficCommunication(); % create instance of the Comunication class
            hlc.scenario.vehicles(index).communicate.traffic = initialize_communication(hlc.scenario.vehicles(index).communicate.traffic, hlc.scenario.vehicles(index).ID); % initialize
            hlc.scenario.vehicles(index).communicate.traffic = create_publisher(hlc.scenario.vehicles(index).communicate.traffic); % create publisher

            hlc.scenario.vehicles(index).communicate.predictions = PredictionsCommunication(); % create instance of the Comunication class
            hlc.scenario.vehicles(index).communicate.predictions = initialize_communication(hlc.scenario.vehicles(index).communicate.predictions, hlc.scenario.vehicles(index).ID); % initialize
            hlc.scenario.vehicles(index).communicate.predictions = create_publisher(hlc.scenario.vehicles(index).communicate.predictions); % create publisher
        end

        duration = toc(start);
        disp(['Finished creating publishers in ' num2str(duration) ' seconds.'])
    end

end
