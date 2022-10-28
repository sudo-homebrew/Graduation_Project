function [data, info] = autoDockingFeedback
%AutoDockingFeedback gives an empty data for kobuki_msgs/AutoDockingFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'kobuki_msgs/AutoDockingFeedback';
[data.State, info.State] = ros.internal.ros.messages.ros.char('string',0);
[data.Text, info.Text] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'kobuki_msgs/AutoDockingFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'state';
info.MatPath{2} = 'text';
