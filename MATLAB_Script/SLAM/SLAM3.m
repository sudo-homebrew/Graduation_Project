
setenv('ROS_DOMAIN_ID','25')
% ros2('topic','list')


domainID = 25;
n = ros2node("matlab_example_robot",domainID);

imgSub = ros2subscriber(n,"/camera/image_raw","sensor_msgs/Image","Reliability","besteffort","Durability","volatile","Depth",5);

odomSub = ros2subscriber(n,"/odom","nav_msgs/Odometry","Reliability","besteffort","Durability","volatile","Depth",5);

lidarSub = ros2subscriber(n, "/scan","sensor_msgs/LaserScan", "Reliability","besteffort","Durability","volatile","Depth",5);

[velPub, velMsg] = ros2publisher(n,"/cmd_vel","geometry_msgs/Twist","Reliability","besteffort","Durability","volatile","Depth",5);

[posPub, posMsg] = ros2publisher(n,"/odom","nav_msgs/Odometry","Reliability","besteffort","Durability","volatile","Depth",5);


velocity = 0.2;

velMsg.linear.x = 0;
velMsg.linear.y = 0;
velMsg.linear.z = 0;
send(velPub,velMsg)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  SLAM Section  %%
maxLidarRange = 8;
mapResolution = 20;
slamAlg = lidarSLAM(mapResolution, maxLidarRange);


slamAlg.LoopClosureThreshold = 210;  
slamAlg.LoopClosureSearchRadius = 8;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




scanMsg = receive(lidarSub);
figure
rosPlot(scanMsg)

sc = {}

pause(3)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

forwardS(0.7, slamAlg, sc, velMsg, velPub, lidarSub, odomSub, velocity)
leftS(0.5, slamAlg, sc, velMsg, velPub, lidarSub, odomSub, velocity)




clear global pos orient
clear