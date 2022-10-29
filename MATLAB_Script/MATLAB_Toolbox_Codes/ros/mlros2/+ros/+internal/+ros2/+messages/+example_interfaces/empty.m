function [data, info] = empty
%Empty gives an empty data for example_interfaces/Empty

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'example_interfaces/Empty';
info.MessageType = 'example_interfaces/Empty';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
