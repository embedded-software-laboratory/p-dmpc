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
        function modeHandler = ExpertMode(exp, scenario, wheelData)
            modeHandler.axes = wheelData.axes;
            modeHandler.buttons = wheelData.buttons;

            modeHandler.steering = modeHandler.axes(1);
            modeHandler.throttle = modeHandler.axes(3);
            modeHandler.brake = modeHandler.axes(4);
            modeHandler.leftPaddle = modeHandler.buttons(6);
            modeHandler.rightPaddle = modeHandler.buttons(5);

            exp.updateManualControl(modeHandler, scenario);

            modeHandler.scenario = scenario;
        end
    end

end