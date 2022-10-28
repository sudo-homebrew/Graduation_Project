function [data, info] = camSelectRequest
%CamSelect gives an empty data for ardrone_autonomy/CamSelectRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ardrone_autonomy/CamSelectRequest';
[data.Channel, info.Channel] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'ardrone_autonomy/CamSelectRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'channel';
