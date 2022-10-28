function [data, info] = deleteCO2SourceRequest
%DeleteCO2Source gives an empty data for stdr_msgs/DeleteCO2SourceRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'stdr_msgs/DeleteCO2SourceRequest';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'stdr_msgs/DeleteCO2SourceRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'name';
