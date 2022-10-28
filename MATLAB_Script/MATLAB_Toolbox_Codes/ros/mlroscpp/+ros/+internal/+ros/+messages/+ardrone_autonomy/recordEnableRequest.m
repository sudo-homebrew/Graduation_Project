function [data, info] = recordEnableRequest
%RecordEnable gives an empty data for ardrone_autonomy/RecordEnableRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ardrone_autonomy/RecordEnableRequest';
[data.Enable, info.Enable] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'ardrone_autonomy/RecordEnableRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'enable';
