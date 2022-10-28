function [data, info] = nodeletListResponse
%NodeletList gives an empty data for nodelet/NodeletListResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'nodelet/NodeletListResponse';
[data.Nodelets, info.Nodelets] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'nodelet/NodeletListResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'nodelets';
