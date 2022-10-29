function [data, info] = course
%Course gives an empty data for kingfisher_msgs/Course

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'kingfisher_msgs/Course';
[data.Yaw, info.Yaw] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Speed, info.Speed] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'kingfisher_msgs/Course';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'yaw';
info.MatPath{2} = 'speed';
