function [data, info] = nullifyDemandRequest
%NullifyDemand gives an empty data for sr_robot_msgs/NullifyDemandRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/NullifyDemandRequest';
[data.NullifyDemand, info.NullifyDemand] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'sr_robot_msgs/NullifyDemandRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'nullify_demand';
