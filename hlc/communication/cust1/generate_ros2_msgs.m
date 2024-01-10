function generate_ros2_msgs()
    %GENERATE_ROS2_MSGS checks whether custom message types (for vehicle
    %communication) already exist and generates them if not.

    msgList = ros2("msg", "list"); % get all ROS 2 message types
    % reg = ros.internal.CustomMessageRegistry.getInstance('ros2',true)
    % msgList = reg.getMessageList
    [file_path, ~, ~] = fileparts(mfilename('fullpath'));

    if ( ...
            (sum(cellfun(@(c)strcmp(c, 'veh_msgs/Traffic'), msgList)) == 0) ...
            || (sum(cellfun(@(c)strcmp(c, 'veh_msgs/Predictions'), msgList)) == 0) ...
            || (sum(cellfun(@(c)strcmp(c, 'veh_msgs/SolutionCost'), msgList)) == 0) ...
            || (sum(cellfun(@(c)strcmp(c, 'veh_msgs/PlottingInfo'), msgList)) == 0) ...
        )
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
            ros2genmsg(file_path)
        catch ME
            disp(['If all environments for ros2genmsg() are prepared but still failed, try to move the whole folder to a ' ...
                  'shallower path and run again if you use Windows machine, which sadly has a max path limit constraint.'])
            throw(ME)
        end

    end

end
