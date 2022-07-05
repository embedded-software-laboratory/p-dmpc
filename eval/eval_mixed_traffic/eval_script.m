function eval_script()

    wheelNode = ros2node("/wheel");
    gamepadNode = ros2node("/gamepad");
    
    wheel_msg = ros2message('sensor_msgs/Joy');
    gamepad_msg = ros2message('sensor_msgs/Joy');

    disp("Initializing publisher...");
    wheelPub = ros2publisher(wheelNode,'/j0','sensor_msgs/Joy');
    gamepadPub = ros2publisher(gamepadNode,'/j1','sensor_msgs/Joy');
    disp("Done");

    action = false;
    action_sequence = 0;
    counter = 0;
   
    t_start = tic;
    % loop to send input for manual vehicles
    while true
        t = toc(t_start);
        
        if t > 5 || counter > 0

            if ~action
                switch action_sequence
                    case 0
                        disp("started sending sequence: steering right");
                    case 1
                        disp("started sending sequence: steering left");
                    case 2
                        disp("started sending sequence: shift up");         
                    otherwise
                        disp("started sending sequence: shift down");  
                end
                
                counter = 1000;
                action = true;
            end

            switch action_sequence
                case 0
                    % steering right
                    wheel_msg.axes = single([-0.5,0.0,0.0,0.0,0.0,0.0]);
                    wheel_msg.buttons = int32([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]);
            
                    gamepad_msg.axes = single([-0.5,0.0,0.0,0.0,0.0,0.0,0.0,0.0]);
                    gamepad_msg.buttons = int32([0,0,0,0,0,0,0,0,0,0,0]);
                case 1
                    % steering left
                    wheel_msg.axes = single([0.5,0.0,0.0,0.0,0.0,0.0]);
                    wheel_msg.buttons = int32([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]);
            
                    gamepad_msg.axes = single([0.5,0.0,0.0,0.0,0.0,0.0,0.0,0.0]);
                    gamepad_msg.buttons = int32([0,0,0,0,0,0,0,0,0,0,0]);
                case 2
                    % shift up
                    wheel_msg.axes = single([0.0,0.0,0.0,0.0,0.0,0.0]);
                    wheel_msg.buttons = int32([0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]);
            
                    gamepad_msg.axes = single([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]);
                    gamepad_msg.buttons = int32([0,0,0,0,1,0,0,0,0,0,0]);           
                otherwise
                    % shift down
                    wheel_msg.axes = single([0.0,0.0,0.0,0.0,0.0,0.0]);
                    wheel_msg.buttons = int32([0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]);
            
                    gamepad_msg.axes = single([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]);
                    gamepad_msg.buttons = int32([0,0,0,0,0,1,0,0,0,0,0]);     
            end

            send(wheelPub, wheel_msg);
            send(gamepadPub, gamepad_msg);
            counter = counter - 1;

            if counter == 0
                disp("stopped sending sequence");
                t_start = tic;
                action = false;
                action_sequence = mod((action_sequence + 1),4);
            end
        else
            wheel_msg.axes = single([0.0,0.0,0.0,0.0,0.0,0.0]);
            wheel_msg.buttons = int32([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]);
            send(wheelPub, wheel_msg);

            gamepad_msg.axes = single([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]);
            gamepad_msg.buttons = int32([0,0,0,0,0,0,0,0,0,0,0]);
            send(gamepadPub, gamepad_msg);
        end
    end

end