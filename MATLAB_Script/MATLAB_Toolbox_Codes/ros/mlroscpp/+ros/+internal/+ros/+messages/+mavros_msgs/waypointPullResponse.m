function [data, info] = waypointPullResponse
%WaypointPull gives an empty data for mavros_msgs/WaypointPullResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/WaypointPullResponse';
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.WpReceived, info.WpReceived] = ros.internal.ros.messages.ros.default_type('uint32',1);
info.MessageType = 'mavros_msgs/WaypointPullResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'success';
info.MatPath{2} = 'wp_received';
