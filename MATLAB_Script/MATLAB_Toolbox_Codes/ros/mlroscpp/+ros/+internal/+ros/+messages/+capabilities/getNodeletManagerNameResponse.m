function [data, info] = getNodeletManagerNameResponse
%GetNodeletManagerName gives an empty data for capabilities/GetNodeletManagerNameResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'capabilities/GetNodeletManagerNameResponse';
[data.NodeletManagerName, info.NodeletManagerName] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'capabilities/GetNodeletManagerNameResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'nodelet_manager_name';
