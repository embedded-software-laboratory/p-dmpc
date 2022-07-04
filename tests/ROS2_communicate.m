options = struct("History","keeplast","Depth",40,"Durability","transientlocal");
msg_type = 'std_msgs/Float64MultiArray';
name1 = '/veh1';
name2 = '/veh2';
node1 = ros2node(name1);
pub1 = ros2publisher(node1,name1,msg_type,options);

%%
subself = ros2subscriber(node1,name1,msg_type,options);
sub1 = ros2subscriber(node1,name2,msg_type,options);

%%
msg1 = ros2message(msg_type);
msg1.data = 33;
send(pub1,msg1)