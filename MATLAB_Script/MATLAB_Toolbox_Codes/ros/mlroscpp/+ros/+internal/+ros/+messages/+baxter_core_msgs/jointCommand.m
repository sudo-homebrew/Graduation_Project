function [data, info] = jointCommand
%JointCommand gives an empty data for baxter_core_msgs/JointCommand

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'baxter_core_msgs/JointCommand';
[data.Mode, info.Mode] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Command, info.Command] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.Names, info.Names] = ros.internal.ros.messages.ros.char('string',NaN);
[data.POSITIONMODE, info.POSITIONMODE] = ros.internal.ros.messages.ros.default_type('int32',1, 1);
[data.VELOCITYMODE, info.VELOCITYMODE] = ros.internal.ros.messages.ros.default_type('int32',1, 2);
[data.TORQUEMODE, info.TORQUEMODE] = ros.internal.ros.messages.ros.default_type('int32',1, 3);
[data.RAWPOSITIONMODE, info.RAWPOSITIONMODE] = ros.internal.ros.messages.ros.default_type('int32',1, 4);
info.MessageType = 'baxter_core_msgs/JointCommand';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'mode';
info.MatPath{2} = 'command';
info.MatPath{3} = 'names';
info.MatPath{4} = 'POSITION_MODE';
info.MatPath{5} = 'VELOCITY_MODE';
info.MatPath{6} = 'TORQUE_MODE';
info.MatPath{7} = 'RAW_POSITION_MODE';
