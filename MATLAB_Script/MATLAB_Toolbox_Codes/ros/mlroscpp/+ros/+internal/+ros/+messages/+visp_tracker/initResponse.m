function [data, info] = initResponse
%Init gives an empty data for visp_tracker/InitResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'visp_tracker/InitResponse';
[data.InitializationSucceed, info.InitializationSucceed] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'visp_tracker/InitResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'initialization_succeed';
