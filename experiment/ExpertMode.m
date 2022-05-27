classdef ExpertMode

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
    end

    methods
        function modeHandler = ExpertMode(exp, scenario, steeringWheel, vehicle_id)

            if steeringWheel
                global stored_wheel_messages_global
                %stored_wheel_messages_global = scenario.ros_subscribers.LatestMessage;

                modeHandler.axes = stored_wheel_messages_global.axes;
                modeHandler.buttons = stored_wheel_messages_global.buttons;
                timestamp = stored_wheel_messages_global.header.stamp.nanosec;
                modeHandler.steering = modeHandler.axes(1);
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

            exp.updateManualControl(modeHandler, scenario, vehicle_id, steeringWheel, timestamp);

            modeHandler.scenario = scenario;
        end
    end

end