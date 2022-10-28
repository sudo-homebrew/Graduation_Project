function [data, info] = setCostmapResponse
%SetCostmap gives an empty data for navfn/SetCostmapResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'navfn/SetCostmapResponse';
info.MessageType = 'navfn/SetCostmapResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
