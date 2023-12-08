function delete_ros2_msgs()
    %DELETE_ROS2_MSGS Deletes the folder into which generate_ros2_msgs()
    %generates the ros2 msg types.

    delete_ros2_msgs_helper("hlc/communication/cust1/matlab_msg_gen");
end

function delete_ros2_msgs_helper(dir_path)

    if exist(dir_path, 'dir')

        try
            fprintf(strcat(dir_path, " exists. Deleting... "));
            rmdir(dir_path, 's');

            % By calling the following command, MATLAB updates its internal
            % MATLAB preferences in which it stores the currently available
            % message types (cf. https://de.mathworks.com/matlabcentral/answers/1691965-ros2-unrecognized-message-without-using-it-in-script).
            % This ensures that ros2("msg", "list") will return up-to-date
            % information, e.g., in generate_ros2_msgs().
            ros.internal.CustomMessageRegistry.getInstance('ros2', true);

            fprintf("done.\n");
        catch
            fprintf("\n");
            warning(strcat("Unable to delete ", dir_path, ". Please delete manually!"));
        end

    end

end
