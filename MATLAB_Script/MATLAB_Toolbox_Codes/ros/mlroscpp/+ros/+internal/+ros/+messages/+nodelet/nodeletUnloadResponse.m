function [data, info] = nodeletUnloadResponse
%NodeletUnload gives an empty data for nodelet/NodeletUnloadResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'nodelet/NodeletUnloadResponse';
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'nodelet/NodeletUnloadResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'success';
