function communication = communication_init(vehicles)
%COMMUNICATION_INIT Initialize the communication network. 
%Create nodes connected to ROS 2 network for communication. One node
%corresponds to one vehicle. Each vehicle has its own topic and sends its
%data only to its own topic.
% Params:
%     vehicles:
%         Vehicles information
% Returns:
%     communication: 
%         All the information (nodes, publishers, subscribers) used for
%         communication.

    nVeh = length(vehicles);

    % connect to ROS network
        
    % create variable to store all the information needed for communication
    communication = cell(nVeh,1);
    
    for iVeh=1:nVeh
        communication{iVeh} = Communication(vehicles(iVeh)); % create instance of the Comunication class
        communication{iVeh} = communication{iVeh}.create_pub_and_subs(nVeh); % create publishers and subscribers
    end
    
    % Each sends its initial data
    for jVeh=1:nVeh
        communication{jVeh}.send_data()
    end

end