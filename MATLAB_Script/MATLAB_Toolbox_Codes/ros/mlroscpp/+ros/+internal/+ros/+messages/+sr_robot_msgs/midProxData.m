function [data, info] = midProxData
%MidProxData gives an empty data for sr_robot_msgs/MidProxData

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/MidProxData';
[data.Middle, info.Middle] = ros.internal.ros.messages.ros.default_type('uint16',4);
[data.Proximal, info.Proximal] = ros.internal.ros.messages.ros.default_type('uint16',4);
info.MessageType = 'sr_robot_msgs/MidProxData';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'middle';
info.MatPath{2} = 'proximal';
