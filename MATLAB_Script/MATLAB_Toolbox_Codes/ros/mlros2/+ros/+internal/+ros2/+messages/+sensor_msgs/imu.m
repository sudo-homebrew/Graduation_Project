function [data, info] = imu
%Imu gives an empty data for sensor_msgs/Imu

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sensor_msgs/Imu';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.orientation, info.orientation] = ros.internal.ros2.messages.geometry_msgs.quaternion;
info.orientation.MLdataType = 'struct';
[data.orientation_covariance, info.orientation_covariance] = ros.internal.ros2.messages.ros2.default_type('double',9,0);
[data.angular_velocity, info.angular_velocity] = ros.internal.ros2.messages.geometry_msgs.vector3;
info.angular_velocity.MLdataType = 'struct';
[data.angular_velocity_covariance, info.angular_velocity_covariance] = ros.internal.ros2.messages.ros2.default_type('double',9,0);
[data.linear_acceleration, info.linear_acceleration] = ros.internal.ros2.messages.geometry_msgs.vector3;
info.linear_acceleration.MLdataType = 'struct';
[data.linear_acceleration_covariance, info.linear_acceleration_covariance] = ros.internal.ros2.messages.ros2.default_type('double',9,0);
info.MessageType = 'sensor_msgs/Imu';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,21);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'orientation';
info.MatPath{7} = 'orientation.x';
info.MatPath{8} = 'orientation.y';
info.MatPath{9} = 'orientation.z';
info.MatPath{10} = 'orientation.w';
info.MatPath{11} = 'orientation_covariance';
info.MatPath{12} = 'angular_velocity';
info.MatPath{13} = 'angular_velocity.x';
info.MatPath{14} = 'angular_velocity.y';
info.MatPath{15} = 'angular_velocity.z';
info.MatPath{16} = 'angular_velocity_covariance';
info.MatPath{17} = 'linear_acceleration';
info.MatPath{18} = 'linear_acceleration.x';
info.MatPath{19} = 'linear_acceleration.y';
info.MatPath{20} = 'linear_acceleration.z';
info.MatPath{21} = 'linear_acceleration_covariance';