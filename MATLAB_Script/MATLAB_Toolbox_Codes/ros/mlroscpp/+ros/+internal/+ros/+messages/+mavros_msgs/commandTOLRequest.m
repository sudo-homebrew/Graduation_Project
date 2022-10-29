function [data, info] = commandTOLRequest
%CommandTOL gives an empty data for mavros_msgs/CommandTOLRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/CommandTOLRequest';
[data.MinPitch, info.MinPitch] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Yaw, info.Yaw] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Latitude, info.Latitude] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Longitude, info.Longitude] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Altitude, info.Altitude] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'mavros_msgs/CommandTOLRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'min_pitch';
info.MatPath{2} = 'yaw';
info.MatPath{3} = 'latitude';
info.MatPath{4} = 'longitude';
info.MatPath{5} = 'altitude';
