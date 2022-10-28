function [data, info] = waypointSetCurrentRequest
%WaypointSetCurrent gives an empty data for mavros_msgs/WaypointSetCurrentRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/WaypointSetCurrentRequest';
[data.WpSeq, info.WpSeq] = ros.internal.ros.messages.ros.default_type('uint16',1);
info.MessageType = 'mavros_msgs/WaypointSetCurrentRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'wp_seq';
