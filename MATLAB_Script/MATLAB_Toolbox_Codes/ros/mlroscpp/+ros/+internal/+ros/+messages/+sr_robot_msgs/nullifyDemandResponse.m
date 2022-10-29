function [data, info] = nullifyDemandResponse
%NullifyDemand gives an empty data for sr_robot_msgs/NullifyDemandResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/NullifyDemandResponse';
info.MessageType = 'sr_robot_msgs/NullifyDemandResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
