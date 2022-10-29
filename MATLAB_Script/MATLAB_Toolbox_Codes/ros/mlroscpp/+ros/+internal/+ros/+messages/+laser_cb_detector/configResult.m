function [data, info] = configResult
%ConfigResult gives an empty data for laser_cb_detector/ConfigResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'laser_cb_detector/ConfigResult';
info.MessageType = 'laser_cb_detector/ConfigResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
