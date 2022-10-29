function [data, info] = robotnikMotorsStatus
%RobotnikMotorsStatus gives an empty data for robotnik_msgs/RobotnikMotorsStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/RobotnikMotorsStatus';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',NaN);
[data.CanId, info.CanId] = ros.internal.ros.messages.ros.default_type('int32',NaN);
[data.MotorStatus, info.MotorStatus] = ros.internal.ros.messages.robotnik_msgs.motorStatus;
info.MotorStatus.MLdataType = 'struct';
info.MotorStatus.MaxLen = NaN;
info.MotorStatus.MinLen = 0;
data.MotorStatus = data.MotorStatus([],1);
info.MessageType = 'robotnik_msgs/RobotnikMotorsStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'name';
info.MatPath{2} = 'can_id';
info.MatPath{3} = 'motor_status';
info.MatPath{4} = 'motor_status.state';
info.MatPath{5} = 'motor_status.status';
info.MatPath{6} = 'motor_status.communicationstatus';
info.MatPath{7} = 'motor_status.statusword';
info.MatPath{8} = 'motor_status.driveflags';
info.MatPath{9} = 'motor_status.activestatusword';
info.MatPath{10} = 'motor_status.activedriveflags';
info.MatPath{11} = 'motor_status.digitaloutputs';
info.MatPath{12} = 'motor_status.digitalinputs';
info.MatPath{13} = 'motor_status.averagecurrent';
info.MatPath{14} = 'motor_status.analoginputs';
