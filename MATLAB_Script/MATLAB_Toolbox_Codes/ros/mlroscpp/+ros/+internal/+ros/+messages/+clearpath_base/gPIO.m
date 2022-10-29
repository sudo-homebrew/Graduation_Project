function [data, info] = gPIO
%GPIO gives an empty data for clearpath_base/GPIO

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'clearpath_base/GPIO';
info.MessageType = 'clearpath_base/GPIO';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
