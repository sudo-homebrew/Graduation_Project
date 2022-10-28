function [data, info] = autoDockingResult
%AutoDockingResult gives an empty data for kobuki_msgs/AutoDockingResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'kobuki_msgs/AutoDockingResult';
[data.Text, info.Text] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'kobuki_msgs/AutoDockingResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'text';
