function [data, info] = loadControllerResponse
%LoadController gives an empty data for pr2_mechanism_msgs/LoadControllerResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_mechanism_msgs/LoadControllerResponse';
[data.Ok, info.Ok] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'pr2_mechanism_msgs/LoadControllerResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'ok';
