function [data, info] = gPADCOutput
%GPADCOutput gives an empty data for clearpath_base/GPADCOutput

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'clearpath_base/GPADCOutput';
info.MessageType = 'clearpath_base/GPADCOutput';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
