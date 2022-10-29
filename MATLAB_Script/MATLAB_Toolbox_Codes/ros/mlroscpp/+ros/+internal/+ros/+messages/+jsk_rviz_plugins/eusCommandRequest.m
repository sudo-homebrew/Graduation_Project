function [data, info] = eusCommandRequest
%EusCommand gives an empty data for jsk_rviz_plugins/EusCommandRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_rviz_plugins/EusCommandRequest';
[data.Command, info.Command] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'jsk_rviz_plugins/EusCommandRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'command';
