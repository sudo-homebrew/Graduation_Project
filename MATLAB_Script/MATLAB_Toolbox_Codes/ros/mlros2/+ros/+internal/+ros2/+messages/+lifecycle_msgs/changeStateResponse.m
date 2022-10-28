function [data, info] = changeStateResponse
%ChangeState gives an empty data for lifecycle_msgs/ChangeStateResponse

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'lifecycle_msgs/ChangeStateResponse';
[data.success, info.success] = ros.internal.ros2.messages.ros2.default_type('logical',1,0);
info.MessageType = 'lifecycle_msgs/ChangeStateResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'success';
