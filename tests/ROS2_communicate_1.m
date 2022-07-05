options = struct("History","keeplast","Depth",40,"Durability","transientlocal");
msg_type = 'std_msgs/Float64MultiArray';
msg_type_3 = 'std_msgs/Byte';
name1 = '/veh1';
name2 = '/veh2';
name3 = '/veh1_helper';
name4 = '/veh2_helper';
node1 = ros2node(name1);
[pub1,msg1] = ros2publisher(node1,name1,msg_type,options);
[pub3,msg3] = ros2publisher(node1,name3,msg_type_3,options);

%%
sub1 = ros2subscriber(node1,name2,msg_type,options);
sub3 = ros2subscriber(node1,name4,msg_type_3,options);

%%
receive(sub1,5)
%%
msg1.data = 1;
send(pub1,msg1)
%%
msg3.data = uint8(3);
send(pub3,msg3)
