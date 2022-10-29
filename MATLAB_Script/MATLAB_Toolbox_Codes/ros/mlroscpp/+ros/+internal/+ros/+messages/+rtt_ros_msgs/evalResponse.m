function [data, info] = evalResponse
%Eval gives an empty data for rtt_ros_msgs/EvalResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rtt_ros_msgs/EvalResponse';
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'rtt_ros_msgs/EvalResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'success';
