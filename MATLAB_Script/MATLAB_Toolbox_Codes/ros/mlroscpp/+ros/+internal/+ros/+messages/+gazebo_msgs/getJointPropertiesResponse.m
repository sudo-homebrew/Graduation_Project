function [data, info] = getJointPropertiesResponse
%GetJointProperties gives an empty data for gazebo_msgs/GetJointPropertiesResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gazebo_msgs/GetJointPropertiesResponse';
[data.Type, info.Type] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.REVOLUTE, info.REVOLUTE] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.CONTINUOUS, info.CONTINUOUS] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.PRISMATIC, info.PRISMATIC] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.FIXED, info.FIXED] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.BALL, info.BALL] = ros.internal.ros.messages.ros.default_type('uint8',1, 4);
[data.UNIVERSAL, info.UNIVERSAL] = ros.internal.ros.messages.ros.default_type('uint8',1, 5);
[data.Damping, info.Damping] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.Position, info.Position] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.Rate, info.Rate] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.StatusMessage, info.StatusMessage] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'gazebo_msgs/GetJointPropertiesResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'type';
info.MatPath{2} = 'REVOLUTE';
info.MatPath{3} = 'CONTINUOUS';
info.MatPath{4} = 'PRISMATIC';
info.MatPath{5} = 'FIXED';
info.MatPath{6} = 'BALL';
info.MatPath{7} = 'UNIVERSAL';
info.MatPath{8} = 'damping';
info.MatPath{9} = 'position';
info.MatPath{10} = 'rate';
info.MatPath{11} = 'success';
info.MatPath{12} = 'status_message';
