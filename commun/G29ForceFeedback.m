classdef G29ForceFeedback

    properties(Access=public)
        g29Pub;             % publisher
        msg_to_be_sent;     % initialize message type
        last_position;
    end

    methods
        function obj = G29ForceFeedback()
        % function to initialize ROS 2 publisher for ros_g29_force_feedback
        
            g29Node = ros2node("/g29");
            obj.last_position = 0.0;
            
            msgList = ros2("msg","list"); % get all ROS 2 message types
            if sum(cellfun(@(c)strcmp(c,'ros_g29_force_feedback/ForceFeedback'), msgList))==0
                
                [file_path,~,~] = fileparts(mfilename('fullpath'));
                path_custom_msg = [file_path,filesep,'custom_msg'];
                
                % Generate custom messages. NOTE that Python 3.7, CMake, and a C++
                % compiler are required (see
                % https://de.mathworks.com/help/ros/gs/ros-system-requirements.html
                % for more details according to your MATLAB version).
                ros2genmsg(path_custom_msg)
            end
            
            setenv LD_LIBRARY_PATH <matlabroot>/extern/bin/glnxa64:<matlabroot>/sys/os/glnxa64
            
            obj.msg_to_be_sent = ros2message('ros_g29_force_feedback/ForceFeedback');
        
            % workaround to be able to create publisher
            obj.g29Pub = ros2publisher(g29Node,"/parameter_events");
            pause(0.2)
        
            % create publisher
            obj.g29Pub = ros2publisher(g29Node,'/ff_target','ros_g29_force_feedback/ForceFeedback');
        end


        function new_last_position = g29_send_message(obj, angle, torque, last_position)
            % send desired position and torque to steering wheel
        
            position = (-1.0) * angle;

            if position == 0.0 && last_position == 0.0
                torque = 0.0;
            end

            obj.msg_to_be_sent.position = single(position);
            obj.msg_to_be_sent.torque = single(torque);
            new_last_position = position;
            send(obj.g29Pub, obj.msg_to_be_sent);
        end
    end

end