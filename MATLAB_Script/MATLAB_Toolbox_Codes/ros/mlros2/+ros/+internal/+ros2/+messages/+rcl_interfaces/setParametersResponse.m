function [data, info] = setParametersResponse
%SetParameters gives an empty data for rcl_interfaces/SetParametersResponse

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'rcl_interfaces/SetParametersResponse';
[data.results, info.results] = ros.internal.ros2.messages.rcl_interfaces.setParametersResult;
info.results.MLdataType = 'struct';
info.results.MaxLen = NaN;
info.results.MinLen = 0;
info.MessageType = 'rcl_interfaces/SetParametersResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'results';
info.MatPath{2} = 'results.successful';
info.MatPath{3} = 'results.reason';
