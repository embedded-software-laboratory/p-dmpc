function callback_when_receiving_message(msg)
% CALLBACK_WHEN_RECEIVING_MESSAGE This is a callback function and will be
% executed automatically every time subscriber of ROS received a message.
%     
% A global variable is defined to store message. Everytime a message is
% received, the publisher and the time step of the message will be checked.
% It will only be stored if no message with the same publisher and time
% step exists.
% A threshold of time step is defined for old messages. Too old messages
% are deleted.

    disp(msg)
    
    global stored_msgs_global

    if isempty(stored_msgs_global)
        % initialize the global variable
        stored_msgs_global = struct('time_step',[],'vehicle_id',[],'state_current',[],'trim_current',[],'predicted_areas',[],'reachable_sets',[],'predicted_lanelets',[],'MessageType',[]);
    end
    
    if msg.time_step==1 && sum([stored_msgs_global.time_step]>1)>0
        % initialize again if there exist messages with time step greater than
        % 1 when the simulation starting (most probably the messages from previous simulation) 
        stored_msgs_global = struct('time_step',[],'vehicle_id',[],'state_current',[],'trim_current',[],'predicted_areas',[],'reachable_sets',[],'predicted_lanelets',[],'MessageType',[]);
    end

    % check the publisher (vehicle ID) and the time step to avoid store the same message 
    find_time_step_idx = find([stored_msgs_global.time_step]==msg.time_step);
    find_veh_ID_idx = find([stored_msgs_global.vehicle_id]==msg.vehicle_id);
    if isempty(intersect(find_time_step_idx,find_veh_ID_idx))
        % no message with the same publisher and time step exists -> store the new message
        if isempty([stored_msgs_global.time_step])
            % if empty
            stored_msgs_global(1) = msg;
        else
            % else append a new row
            stored_msgs_global(end+1) = msg;
        end
    end

    % delete messages older than a certain time steps compared to the time step of newly received message
    threshold_older = 5;
    find_old_msgs = [stored_msgs_global.time_step]<=msg.time_step-threshold_older;
    stored_msgs_global(find_old_msgs) = [];
end