function [data, info] = setCostmapRequest
%SetCostmap gives an empty data for navfn/SetCostmapRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'navfn/SetCostmapRequest';
[data.Costs, info.Costs] = ros.internal.ros.messages.ros.default_type('uint8',NaN);
[data.Height, info.Height] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.Width, info.Width] = ros.internal.ros.messages.ros.default_type('uint16',1);
info.MessageType = 'navfn/SetCostmapRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'costs';
info.MatPath{2} = 'height';
info.MatPath{3} = 'width';
