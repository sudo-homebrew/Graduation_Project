function [data, info] = torsoStatus
%TorsoStatus gives an empty data for r2_msgs/TorsoStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'r2_msgs/TorsoStatus';
[data.Leftarm, info.Leftarm] = ros.internal.ros.messages.ros.char('string',0);
[data.Rightarm, info.Rightarm] = ros.internal.ros.messages.ros.char('string',0);
[data.Neck, info.Neck] = ros.internal.ros.messages.ros.char('string',0);
[data.Lefthand, info.Lefthand] = ros.internal.ros.messages.ros.char('string',0);
[data.Righthand, info.Righthand] = ros.internal.ros.messages.ros.char('string',0);
[data.Waist, info.Waist] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'r2_msgs/TorsoStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'leftarm';
info.MatPath{2} = 'rightarm';
info.MatPath{3} = 'neck';
info.MatPath{4} = 'lefthand';
info.MatPath{5} = 'righthand';
info.MatPath{6} = 'waist';
