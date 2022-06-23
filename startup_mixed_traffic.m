function startup_mixed_traffic(number_of_controllers, force_feedback_enabled)
% STARTUP   Startup packages for mixed traffic scenarios. Call before executing the main function.
% number_of_controllers: specifiy whether a single controller (steering wheel) or two controllers (steering wheel and gamepad) will be used
% force_feedback_enabled: enable sending commands to steering wheel

    if ~isboolean(force_feedback_enabled)
        disp("variable force_feedback_enabled has to be true or false");
        return
    end

    if number_of_controllers == 1 || number_of_controllers == 2
        pathToScript = fullfile(pwd,'/launch','launch_controller_packages.sh');
        cmdStr = [pathToScript ' ' number_of_controllers ' ' force_feedback_enabled];
        system(cmdStr);
    else
        disp("Invalid amount of controllers, enter 1 or 2");
        return
    end
end