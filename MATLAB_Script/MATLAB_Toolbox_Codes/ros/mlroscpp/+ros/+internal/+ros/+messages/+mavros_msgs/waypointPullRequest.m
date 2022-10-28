function [data, info] = waypointPullRequest
%WaypointPull gives an empty data for mavros_msgs/WaypointPullRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/WaypointPullRequest';
info.MessageType = 'mavros_msgs/WaypointPullRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
