function [data, info] = sensorPerformanceMetric
%SensorPerformanceMetric gives an empty data for gazebo_msgs/SensorPerformanceMetric

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gazebo_msgs/SensorPerformanceMetric';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.SimUpdateRate, info.SimUpdateRate] = ros.internal.ros.messages.ros.default_type('double',1);
[data.RealUpdateRate, info.RealUpdateRate] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Fps, info.Fps] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'gazebo_msgs/SensorPerformanceMetric';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'name';
info.MatPath{2} = 'sim_update_rate';
info.MatPath{3} = 'real_update_rate';
info.MatPath{4} = 'fps';
