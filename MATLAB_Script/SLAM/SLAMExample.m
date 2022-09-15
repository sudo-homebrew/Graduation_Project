ipaddress = '192.168.188.128';
rosinit(ipaddress,11311);

% rostopic list;

velocity = 0.2;

robotCmd = rospublisher("/cmd_vel","DataFormat","struct") ;
velMsg = rosmessage(robotCmd);

velMsg.Linear.X = velocity;
send(robotCmd,velMsg);
pause(4);
velMsg.Linear.X = 0;
send(robotCmd,velMsg);



rostopic type /cmd_vel;

rostopic info /cmd_vel


odomSub = rossubscriber("/odom","DataFormat","struct");

odomMsg = receive(odomSub,3);
pose = odomMsg.Pose.Pose;
x = pose.Position.X;
y = pose.Position.Y;
z = pose.Position.Z;

% [x y z]

% quat = pose.Orientation;
% angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
% theta = rad2deg(angles(1))


pause(5);

%%%%%%  Recieve Lidar Data

lidarSub = rossubscriber("/scan", "DataFormat", "struct");


scanMsg = receive(lidarSub);
figure
rosPlot(scanMsg)



%%%%%%  Recieve Lidar Data (While it's moving)

velMsg.Angular.Z = velocity;
send(robotCmd,velMsg)
tic;
while toc < 10
  scanMsg = receive(lidarSub);
  rosPlot(scanMsg)
end


velMsg.Angular.Z = 0;
send(robotCmd,velMsg)



velMsg.Linear.X = velocity;
send(robotCmd,velMsg)
tic;

while toc < 20
  scanMsg = receive(lidarSub);
  rosPlot(scanMsg)
end

velMsg.Linear.X = 0;
send(robotCmd,velMsg)

%%%%%% Disconnecting ROS
% clear all;

rosshutdown;