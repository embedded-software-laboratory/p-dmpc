options = struct("History","keeplast","Depth",40,"Durability","transientlocal");
msg_type = 'std_msgs/Float64MultiArray';
name1 = '/veh1';
name2 = '/veh2';
node2 = ros2node(name2);
pub2 = ros2publisher(node2,name2,msg_type,options);

%%
sub2 = ros2subscriber(node2,name1,msg_type,options);

%%
receive(sub2,5)
%%
msg2 = ros2message(msg_type);
msg2.data = 2;
send(pub2,msg2)