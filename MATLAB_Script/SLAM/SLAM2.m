

% function success = straight(sec)
%     velMsg.linear.x = velocity;
%     send(velPub,velMsg)
%     tic;
%     
%     while toc < sec
%       scanMsg = receive(lidarSub);
%       rosPlot(scanMsg)
%       sc_temp = rosReadLidarScan(scanMsg);
%       sc = [sc, sc_temp];
%       [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, sc_tewmp)
%     end
%     
%     velMsg.linear.x = 0;
%     send(velPub,velMsg)
% 
%     success = true;
% end




setenv('ROS_DOMAIN_ID','25')
% ros2('topic','list')

domainID = 25;
n = ros2node("matlab_example_robot",domainID);

imgSub = ros2subscriber(n,"/camera/image_raw","sensor_msgs/Image","Reliability","besteffort","Durability","volatile","Depth",5);

odomSub = ros2subscriber(n,"/odom","nav_msgs/Odometry","Reliability","besteffort","Durability","volatile","Depth",5);

[velPub, velMsg] = ros2publisher(n,"/cmd_vel","geometry_msgs/Twist","Reliability","besteffort","Durability","volatile","Depth",5);


velocity = 0.2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  SLAM Section  %%
maxLidarRange = 8;
mapResolution = 20;
slamAlg = lidarSLAM(mapResolution, maxLidarRange);


slamAlg.LoopClosureThreshold = 210;  
slamAlg.LoopClosureSearchRadius = 8;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

lidarSub = ros2subscriber(n, "/scan","sensor_msgs/LaserScan", "Reliability","besteffort","Durability","volatile","Depth",5);


scanMsg = receive(lidarSub);
figure
rosPlot(scanMsg)

sc = {}



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55



velMsg.linear.x = velocity;
send(velPub,velMsg)
tic;

while toc < 5
  scanMsg = receive(lidarSub);
  rosPlot(scanMsg)
  sc_temp = rosReadLidarScan(scanMsg);
  sc = [sc, sc_temp];
  [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, sc_temp);
end

velMsg.linear.x = 0;
send(velPub,velMsg)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

velMsg.angular.z = velocity;
send(velPub,velMsg)
tic;

while toc < 9
  scanMsg = receive(lidarSub);
  rosPlot(scanMsg)
  sc_temp = rosReadLidarScan(scanMsg);
  sc = [sc, sc_temp];
  [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, sc_temp);
end

velMsg.angular.z = 0;
send(velPub,velMsg)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



velMsg.linear.x = velocity;
send(velPub,velMsg)
tic;

while toc < 7
  scanMsg = receive(lidarSub);
  rosPlot(scanMsg)
  sc_temp = rosReadLidarScan(scanMsg);
  sc = [sc, sc_temp];
  [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, sc_temp);
end

velMsg.linear.x = 0;
send(velPub,velMsg)




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




velMsg.angular.z = -velocity;
send(velPub,velMsg)
tic;

while toc < 9
  scanMsg = receive(lidarSub);
  rosPlot(scanMsg)
  sc_temp = rosReadLidarScan(scanMsg);
  sc = [sc, sc_temp];
  [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, sc_temp);
end

velMsg.angular.z = 0;
send(velPub,velMsg)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



velMsg.linear.x = velocity;
send(velPub,velMsg)
tic;

while toc < 9
  scanMsg = receive(lidarSub);
  rosPlot(scanMsg)
  sc_temp = rosReadLidarScan(scanMsg);
  sc = [sc, sc_temp];
  [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, sc_temp);
end

velMsg.linear.x = 0;
send(velPub,velMsg)




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

velMsg.angular.z = velocity;
send(velPub,velMsg)
tic;

while toc < 9
  scanMsg = receive(lidarSub);
  rosPlot(scanMsg)
  sc_temp = rosReadLidarScan(scanMsg);
  sc = [sc, sc_temp];
  [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, sc_temp);
end

velMsg.angular.z = 0;
send(velPub,velMsg)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



velMsg.linear.x = velocity;
send(velPub,velMsg)
tic;

while toc < 10
  scanMsg = receive(lidarSub);
  rosPlot(scanMsg)
  sc_temp = rosReadLidarScan(scanMsg);
  sc = [sc, sc_temp];
  [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, sc_temp);
end

velMsg.linear.x = 0;
send(velPub,velMsg)




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



figure;
show(slamAlg);
title({'Map of the Environment','Pose Graph'});




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



[scans, optimizedPoses]  = scansAndPoses(slamAlg);
map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);



figure; 
show(map);
hold on
show(slamAlg.PoseGraph, 'IDs', 'off');
hold off
title('Occupancy Grid Map Built Using Lidar SLAM');



clear global pos orient
clear

