function [data, info] = getFirstMapResult
%GetFirstMapResult gives an empty data for nav2d_navigator/GetFirstMapResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'nav2d_navigator/GetFirstMapResult';
info.MessageType = 'nav2d_navigator/GetFirstMapResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
