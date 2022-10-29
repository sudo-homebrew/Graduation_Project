function [data, info] = nodeletListRequest
%NodeletList gives an empty data for nodelet/NodeletListRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'nodelet/NodeletListRequest';
info.MessageType = 'nodelet/NodeletListRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
