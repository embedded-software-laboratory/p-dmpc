function callback_traffic_communication(msg)
    % CALLBACK_WHEN_RECEIVING_MESSAGE This is a callback function and will be
    % executed automatically every time subscriber of ROS received a message.
    %
    % A global variable is defined to store message. Everytime a message is
    % received, the publisher and the time step of the message will be checked.
    % A threshold of time step is defined for old messages. Too old messages
    % are deleted.

    global stored_traffic_msgs

    if isempty(stored_traffic_msgs)
        % if empty (no messages received so far)
        stored_traffic_msgs = msg;
    else
        % else check if message with same vehicle id and time step exists
        % and delete this message then
        is_msg_outdated = ...
            ([stored_traffic_msgs.vehicle_id] == msg.vehicle_id) & ...
            ([stored_traffic_msgs.time_step] == msg.time_step);
        stored_traffic_msgs(is_msg_outdated) = [];
        % append message to list
        stored_traffic_msgs(end + 1) = msg;
    end

    % delete messages older than a certain time steps compared to the time step of newly received message
    message_age_threshold = 2;
    is_msg_expired = [stored_traffic_msgs.time_step] <= (msg.time_step - message_age_threshold);
    stored_traffic_msgs(is_msg_expired) = [];
end
