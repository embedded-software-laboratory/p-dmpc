function distance = check_distance(iter,veh1, veh2)

    veh1_current_position_x = iter.x0(veh1,1);
    veh1_current_position_y = iter.x0(veh1,2);
    veh1_current_position = [veh1_current_position_x,veh1_current_position_y];

    veh2_current_position_x = iter.x0(veh2,1);
    veh2_current_position_y = iter.x0(veh2,2);
    veh2_current_position = [veh2_current_position_x,veh2_current_position_y];
    
    distance = norm((veh1_current_position - veh2_current_position),2);

end