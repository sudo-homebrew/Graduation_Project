function [data, info] = getParameterTypesResponse
%GetParameterTypes gives an empty data for rcl_interfaces/GetParameterTypesResponse

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'rcl_interfaces/GetParameterTypesResponse';
[data.types, info.types] = ros.internal.ros2.messages.ros2.default_type('uint8',NaN,0);
info.MessageType = 'rcl_interfaces/GetParameterTypesResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'types';
