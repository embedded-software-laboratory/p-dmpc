function ula_ros2gen
    % generate custom message type (for communication with unified lab api) if not exist
    msgList = ros2("msg", "list"); % get all ROS 2 message types

    if ((sum(cellfun(@(c)strcmp(c, 'ula_interfaces/Ready'), msgList)) == 0))
        % if the message type 'veh_msgs/Traffic' does not exist
        [file_path, ~, ~] = fileparts(mfilename('fullpath'));

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
        disp('Generating ROS 2 custom message type for unified lab interface...')

        try
            ros2genmsg(file_path)
        catch ME
            disp(['If all environments for ros2genmsg() are prepared but still failed, try to move the whole folder to a ' ...
                  'shallower path and run again if you use Windows machine, which sadly has a max path limit constraint.'])
            throw(ME)
        end

    else
        disp(['No generation of ROS 2 custom message type for unified lab interface, since at least "ula_interfaces/Ready" ' ...
              'message exists. If message types are missing regenerate all by removing the folder matlab_msg_gen...'])
    end

end
