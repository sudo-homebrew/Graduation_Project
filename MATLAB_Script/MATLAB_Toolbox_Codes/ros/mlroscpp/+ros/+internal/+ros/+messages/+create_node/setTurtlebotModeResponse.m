function [data, info] = setTurtlebotModeResponse
%SetTurtlebotMode gives an empty data for create_node/SetTurtlebotModeResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'create_node/SetTurtlebotModeResponse';
[data.ValidMode, info.ValidMode] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'create_node/SetTurtlebotModeResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'valid_mode';
