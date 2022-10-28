function [data, info] = drive
%Drive gives an empty data for kingfisher_msgs/Drive

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'kingfisher_msgs/Drive';
[data.Left, info.Left] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Right, info.Right] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'kingfisher_msgs/Drive';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'left';
info.MatPath{2} = 'right';
