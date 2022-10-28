function [data, info] = jointStatus
%JointStatus gives an empty data for r2_msgs/JointStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'r2_msgs/JointStatus';
[data.Publisher, info.Publisher] = ros.internal.ros.messages.ros.char('string',0);
[data.Joint, info.Joint] = ros.internal.ros.messages.ros.char('string',0);
[data.RegisterValue, info.RegisterValue] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.BridgeEnabled, info.BridgeEnabled] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.MotorEnabled, info.MotorEnabled] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.BrakeReleased, info.BrakeReleased] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.MotorPowerDetected, info.MotorPowerDetected] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.EmbeddedMotCom, info.EmbeddedMotCom] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.JointFaulted, info.JointFaulted] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'r2_msgs/JointStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'publisher';
info.MatPath{2} = 'joint';
info.MatPath{3} = 'registerValue';
info.MatPath{4} = 'bridgeEnabled';
info.MatPath{5} = 'motorEnabled';
info.MatPath{6} = 'brakeReleased';
info.MatPath{7} = 'motorPowerDetected';
info.MatPath{8} = 'embeddedMotCom';
info.MatPath{9} = 'jointFaulted';
