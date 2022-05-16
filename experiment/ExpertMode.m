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
        function modeHandler = ExpertMode(exp, scenario, inputData, steeringWheel)
            modeHandler.axes = inputData.axes;
            modeHandler.buttons = inputData.buttons;

            if steeringWheel
                modeHandler.steering = modeHandler.axes(1);
                modeHandler.throttle = modeHandler.axes(3);
                modeHandler.brake = modeHandler.axes(4);
                modeHandler.leftPaddle = modeHandler.buttons(6);
                modeHandler.rightPaddle = modeHandler.buttons(5);
            else
                modeHandler.steering = modeHandler.axes(1);
                modeHandler.throttle = (-1) * modeHandler.axes(3);
                modeHandler.brake = (-1) * modeHandler.axes(6);
                modeHandler.leftPaddle = modeHandler.buttons(5);
                modeHandler.rightPaddle = modeHandler.buttons(6);
            end

            exp.updateManualControl(modeHandler, scenario);

            modeHandler.scenario = scenario;
        end
    end

end