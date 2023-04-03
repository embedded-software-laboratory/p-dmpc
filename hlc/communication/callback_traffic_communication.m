function callback_traffic_communication(msg)
    % CALLBACK_WHEN_RECEIVING_MESSAGE This is a callback function and will be
    % executed automatically every time subscriber of ROS received a message.
    %
    % A global variable is defined to store message. Everytime a message is
    % received, the publisher and the time step of the message will be checked.
    % It will only be stored if no message with the same publisher and time
    % step exists.
    % A threshold of time step is defined for old messages. Too old messages
    % are deleted.

    global stored_traffic_msgs

    %     if msg.time_step==1 && sum([stored_msgs_global.time_step]>1)>0
    %         % initialize again if there exist messages with time step greater than
    %         % 1 when the simulation starting (most probably the messages from previous simulation)
    %         stored_msgs_global = struct('time_step',[],'vehicle_id',[],'state_current',[],'trim_current',[],'predicted_areas',[],'reachable_sets',[],'predicted_lanelets',[],'MessageType',[]);
    %     end

    % no message with the same publisher and time step exists -> store the new message
    if isempty([stored_traffic_msgs.time_step])
        % if empty (no messages received so far)
        stored_traffic_msgs(1) = msg;
    else
        veh_id_msg = find([stored_traffic_msgs.vehicle_id] == msg.vehicle_id);
        time_step_msg = find([stored_traffic_msgs.time_step] == msg.time_step);
        outdated_msg = intersect(veh_id_msg, time_step_msg);
        stored_traffic_msgs(outdated_msg) = [];
        % else append a new row
        stored_traffic_msgs(end + 1) = msg;
    end

    % delete messages older than a certain time steps compared to the time step of newly received message
    threshold_older = 2;
    find_old_msgs = [stored_traffic_msgs.time_step] <= msg.time_step - threshold_older;
    stored_traffic_msgs(find_old_msgs) = [];
end
