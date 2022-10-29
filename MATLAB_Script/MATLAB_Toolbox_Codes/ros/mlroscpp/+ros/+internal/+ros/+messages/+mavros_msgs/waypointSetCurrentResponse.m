function [data, info] = waypointSetCurrentResponse
%WaypointSetCurrent gives an empty data for mavros_msgs/WaypointSetCurrentResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/WaypointSetCurrentResponse';
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'mavros_msgs/WaypointSetCurrentResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'success';
