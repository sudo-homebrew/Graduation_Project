function [data, info] = recordEnableResponse
%RecordEnable gives an empty data for ardrone_autonomy/RecordEnableResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ardrone_autonomy/RecordEnableResponse';
[data.Result, info.Result] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'ardrone_autonomy/RecordEnableResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'result';
