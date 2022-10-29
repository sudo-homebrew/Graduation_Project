function [data, info] = jointControl
%JointControl gives an empty data for r2_msgs/JointControl

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'r2_msgs/JointControl';
[data.Publisher, info.Publisher] = ros.internal.ros.messages.ros.char('string',0);
[data.TimeStamp, info.TimeStamp] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Joint, info.Joint] = ros.internal.ros.messages.ros.char('string',0);
[data.RegisterValue, info.RegisterValue] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.EnableBridge, info.EnableBridge] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.EnableMotor, info.EnableMotor] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.ReleaseBrake, info.ReleaseBrake] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.EmbeddedMotCom, info.EmbeddedMotCom] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.ControlMode, info.ControlMode] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.ClearFaults, info.ClearFaults] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'r2_msgs/JointControl';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'publisher';
info.MatPath{2} = 'timeStamp';
info.MatPath{3} = 'joint';
info.MatPath{4} = 'registerValue';
info.MatPath{5} = 'enableBridge';
info.MatPath{6} = 'enableMotor';
info.MatPath{7} = 'releaseBrake';
info.MatPath{8} = 'embeddedMotCom';
info.MatPath{9} = 'controlMode';
info.MatPath{10} = 'clearFaults';
