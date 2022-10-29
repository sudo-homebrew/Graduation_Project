function [data, info] = allowedCollisionEntry
%AllowedCollisionEntry gives an empty data for moveit_msgs/AllowedCollisionEntry

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/AllowedCollisionEntry';
[data.Enabled, info.Enabled] = ros.internal.ros.messages.ros.default_type('logical',NaN);
info.MessageType = 'moveit_msgs/AllowedCollisionEntry';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'enabled';
