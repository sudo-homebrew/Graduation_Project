function [data, info] = setTurtlebotModeRequest
%SetTurtlebotMode gives an empty data for create_node/SetTurtlebotModeRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'create_node/SetTurtlebotModeRequest';
[data.Mode, info.Mode] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'create_node/SetTurtlebotModeRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'mode';
