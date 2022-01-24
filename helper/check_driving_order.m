function is_leading_vehicle = check_driving_order( iter, nveh, iveh)

        current_point_x = iter.x0(nveh,1);
        current_point_y = iter.x0(nveh,2);
        current_point = [current_point_x,current_point_y];


        first_refPoint_x = iter.referenceTrajectoryPoints(nveh,1,1);
        first_refPoint_y = iter.referenceTrajectoryPoints(nveh,1,2);
        first_refPoint = [first_refPoint_x,first_refPoint_y];

        second_refPoint_x = iter.referenceTrajectoryPoints(nveh,end,1);
        second_refPoint_y = iter.referenceTrajectoryPoints(nveh,end,2);
        second_refPoint = [second_refPoint_x,second_refPoint_y];

%         driving_direction = first_refPoint - current_point;
        driving_direction = second_refPoint - first_refPoint;

        % reference trajectory points of adjacent vehicles
        current_point_adj_x = iter.x0(iveh,1);
        current_point_adj_y = iter.x0(iveh,2);
        current_point_adj = [current_point_adj_x,current_point_adj_y];

        first_refPoint_adj_x = iter.referenceTrajectoryPoints(iveh,1,1);
        first_refPoint_adj_y = iter.referenceTrajectoryPoints(iveh,1,2);
        first_refPoint_adj = [first_refPoint_adj_x,first_refPoint_adj_y];

        second_refPoint_adj_x = iter.referenceTrajectoryPoints(iveh,end,1);
        second_refPoint_adj_y = iter.referenceTrajectoryPoints(iveh,end,2);
        second_refPoint_adj = [second_refPoint_adj_x ,second_refPoint_adj_y];

%         driving_direction_adj = first_refPoint_adj - current_point_adj;
        driving_direction_adj = second_refPoint_adj - first_refPoint_adj;

        driving_direction_ref = (driving_direction + driving_direction_adj)/2;


%         vehicle_relative_direction = current_point - current_point_adj;
        vehicle_relative_direction = first_refPoint - first_refPoint_adj;

        % check if driving consecutively
        is_leading_vehicle = (dot(driving_direction_ref, vehicle_relative_direction) > 0);


end