function communicate_init_state(scenario)
%COMMUNICATE_INIT_STATE Vehicles communicate initial states using ROS
    
for i=1:scenario.nVeh
    traffic_info = struct;
    traffic_info.vehicle = scenario.vehicles(1,i);
    traffic_info.vehicle_to_lanelet = scenario.vehicle_to_lanelet(i,:);
    

end