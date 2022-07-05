options = struct("History","keeplast","Depth",40,"Durability","transientlocal");
msg_type = 'std_msgs/Float64MultiArray';
msg_type_4 = 'std_msgs/Byte';
name1 = '/veh1';
name2 = '/veh2';
name3 = '/veh1_helper';
name4 = '/veh2_helper';
node2 = ros2node(name2);
[pub2,msg2] = ros2publisher(node2,name2,msg_type,options);
[pub4,msg4] = ros2publisher(node2,name4,msg_type_4,options);

%%
sub2 = ros2subscriber(node2,name1,msg_type,options);
sub4 = ros2subscriber(node2,name3,msg_type_4,options);

%%
receive(sub2,5)
%%
receive(sub4,5)
%%
msg2.data = 222;
send(pub2,msg2)
%%
msg4.data = uint(44);
send(pub4,msg4)