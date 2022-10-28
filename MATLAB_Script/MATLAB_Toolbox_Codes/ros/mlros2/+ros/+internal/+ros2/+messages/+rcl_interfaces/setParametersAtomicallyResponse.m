function [data, info] = setParametersAtomicallyResponse
%SetParametersAtomically gives an empty data for rcl_interfaces/SetParametersAtomicallyResponse

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'rcl_interfaces/SetParametersAtomicallyResponse';
[data.result, info.result] = ros.internal.ros2.messages.rcl_interfaces.setParametersResult;
info.result.MLdataType = 'struct';
info.MessageType = 'rcl_interfaces/SetParametersAtomicallyResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'result';
info.MatPath{2} = 'result.successful';
info.MatPath{3} = 'result.reason';
