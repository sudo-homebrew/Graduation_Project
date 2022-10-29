function [data, info] = getInitRequest
%GetInit gives an empty data for interactive_marker_proxy/GetInitRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'interactive_marker_proxy/GetInitRequest';
info.MessageType = 'interactive_marker_proxy/GetInitRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
