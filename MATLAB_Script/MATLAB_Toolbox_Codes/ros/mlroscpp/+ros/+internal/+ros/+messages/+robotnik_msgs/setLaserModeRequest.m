function [data, info] = setLaserModeRequest
%SetLaserMode gives an empty data for robotnik_msgs/SetLaserModeRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/SetLaserModeRequest';
[data.Mode, info.Mode] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'robotnik_msgs/SetLaserModeRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'mode';
