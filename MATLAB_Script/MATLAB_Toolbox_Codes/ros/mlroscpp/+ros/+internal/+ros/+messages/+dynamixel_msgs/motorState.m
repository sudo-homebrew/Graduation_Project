function [data, info] = motorState
%MotorState gives an empty data for dynamixel_msgs/MotorState

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'dynamixel_msgs/MotorState';
[data.Timestamp, info.Timestamp] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Id, info.Id] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Goal, info.Goal] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Position, info.Position] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Error, info.Error] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Speed, info.Speed] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Load, info.Load] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Voltage, info.Voltage] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Temperature, info.Temperature] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Moving, info.Moving] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'dynamixel_msgs/MotorState';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'timestamp';
info.MatPath{2} = 'id';
info.MatPath{3} = 'goal';
info.MatPath{4} = 'position';
info.MatPath{5} = 'error';
info.MatPath{6} = 'speed';
info.MatPath{7} = 'load';
info.MatPath{8} = 'voltage';
info.MatPath{9} = 'temperature';
info.MatPath{10} = 'moving';
