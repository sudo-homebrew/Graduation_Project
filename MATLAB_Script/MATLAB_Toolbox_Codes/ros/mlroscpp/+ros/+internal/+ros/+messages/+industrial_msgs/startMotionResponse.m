function [data, info] = startMotionResponse
%StartMotion gives an empty data for industrial_msgs/StartMotionResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'industrial_msgs/StartMotionResponse';
[data.Code, info.Code] = ros.internal.ros.messages.industrial_msgs.serviceReturnCode;
info.Code.MLdataType = 'struct';
info.MessageType = 'industrial_msgs/StartMotionResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'code';
info.MatPath{2} = 'code.val';
info.MatPath{3} = 'code.SUCCESS';
info.MatPath{4} = 'code.FAILURE';
