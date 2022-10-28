function [data, info] = setDrivePowerResponse
%SetDrivePower gives an empty data for industrial_msgs/SetDrivePowerResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'industrial_msgs/SetDrivePowerResponse';
[data.Code, info.Code] = ros.internal.ros.messages.industrial_msgs.serviceReturnCode;
info.Code.MLdataType = 'struct';
info.MessageType = 'industrial_msgs/SetDrivePowerResponse';
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
