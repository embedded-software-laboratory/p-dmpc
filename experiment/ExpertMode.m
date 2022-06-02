%classdef ExpertMode
    %{
    properties(Access=public)
        axes
        buttons
        steering
        throttle
        brake
        leftPaddle
        rightPaddle
        scenario
        updatedPath
        mpa
        wheelNode
        wheelSub
    end
    %}

    %methods
    function modeHandler = ExpertMode(exp, scenario, steeringWheel, vehicle_id)
        %function modeHandler = ExpertMode(exp, scenario, steeringWheel, vehicle_id, inputData)
        %function modeHandler = ExpertMode(input)
            %exp = input.exp;
            %scenario = input.scenario;
            %steeringWheel = input.steeringWheel;
            %vehicle_id = input.vehicle_id;

            modeHandler = struct;
            modeHandler.counter = 1;
            %modeHandler.wheelNode = parallel.pool.Constant(@() ros2node("/wheel"));
            %modeHandler.wheelSub = parallel.pool.Constant(@() ros2subscriber(modeHandler.wheelNode,"/j0","sensor_msgs/Joy"));
            %modeHandler.wheelNode = ros2node("/wheel");
            %modeHandler.wheelSub = ros2subscriber(modeHandler.wheelNode,"/j0","sensor_msgs/Joy",@obj.steeringWheelCallback);
            
                %inputData = receive(modeHandler.wheelSub, 1);
                %inputData = exp.getWheelData();

                if steeringWheel
                    disp("reached");
                    %global stored_wheel_messages_global
                    stored_wheel_messages_global = exp.wheelSub.LatestMessage;

                    
                    modeHandler.axes = stored_wheel_messages_global.axes;
                    modeHandler.buttons = stored_wheel_messages_global.buttons;
                    timestamp = stored_wheel_messages_global.header.stamp.nanosec;
                    %modeHandler.axes = inputData.axes;
                    %modeHandler.buttons = inputData.buttons;
                    modeHandler.steering =  modeHandler.axes(1);
                    disp(modeHandler.steering);
                    modeHandler.throttle = modeHandler.axes(3);
                    modeHandler.brake = modeHandler.axes(4);
                    modeHandler.leftPaddle = modeHandler.buttons(6);
                    modeHandler.rightPaddle = modeHandler.buttons(5);
                else
                    modeHandler.steering = modeHandler.axes(1);
                    modeHandler.throttle = (-1) * modeHandler.axes(6);
                    modeHandler.brake = (-1) * modeHandler.axes(3);
                    modeHandler.leftPaddle = modeHandler.buttons(5);
                    modeHandler.rightPaddle = modeHandler.buttons(6);
                end

                exp.updateManualControl(modeHandler, scenario, vehicle_id, steeringWheel);

                modeHandler.scenario = scenario;
            
        end
    %end

%end