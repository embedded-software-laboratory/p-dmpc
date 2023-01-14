classdef G29ForceFeedback

    properties(Access = private)
        g29_pub;             % publisher
        msg_to_be_sent;     % initialize message type
    end

    methods
        function obj = G29ForceFeedback()
        % function to initialize ROS 2 publisher for ros_g29_force_feedback
        
            g29_node = ros2node("/g29");
            
            msgList = ros2("msg","list"); % get all ROS 2 message types
            if sum(cellfun(@(c)strcmp(c,'ros_g29_force_feedback/ForceFeedback'), msgList))==0
                
                [file_path,~,~] = fileparts(mfilename('fullpath'));
                path_custom_msg = [file_path,filesep,'../commun/cust2'];
                
                % Generate custom messages. NOTE that Python 3.7, CMake, and a C++
                % compiler are required (see
                % https://de.mathworks.com/help/ros/gs/ros-system-requirements.html
                % for more details according to your MATLAB version).
                ros2genmsg(path_custom_msg)
            end
            
            setenv LD_LIBRARY_PATH <matlabroot>/extern/bin/glnxa64:<matlabroot>/sys/os/glnxa64
            
            obj.msg_to_be_sent = ros2message('ros_g29_force_feedback/ForceFeedback');
        
            obj.g29_pub = ros2publisher(g29_node,'/ff_target','ros_g29_force_feedback/ForceFeedback');
        end


        function send_message(obj, data)
            % send desired position and torque to steering wheel
            obj.msg_to_be_sent.position = single(data.position);
            obj.msg_to_be_sent.torque = single(data.torque);
            send(obj.g29_pub, obj.msg_to_be_sent);
        end
    end
    methods (Static)
        function last_position = get_last_position()
            last_position = obj.last_position;
        end
    end

end