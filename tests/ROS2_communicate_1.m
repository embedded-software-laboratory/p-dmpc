options = struct("History","keeplast","Depth",40,"Durability","transientlocal");
msg_type = 'std_msgs/Float64MultiArray';
name = 'veh1';
node1 = ros2node(name);
pub1 = ros2publisher(node1,name,msg_type,options);

%%
sub1 = ros2subscriber(node1,name,msg_type,options);

%%
msg1 = ros2message(msg_type);
msg1.data = 1;
send(pub1,msg1)