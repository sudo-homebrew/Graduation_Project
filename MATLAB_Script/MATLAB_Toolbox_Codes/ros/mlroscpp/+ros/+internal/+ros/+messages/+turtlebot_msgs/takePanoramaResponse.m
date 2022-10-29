function [data, info] = takePanoramaResponse
%TakePanorama gives an empty data for turtlebot_msgs/TakePanoramaResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'turtlebot_msgs/TakePanoramaResponse';
[data.Status, info.Status] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.STARTED, info.STARTED] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.INPROGRESS, info.INPROGRESS] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.STOPPED, info.STOPPED] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
info.MessageType = 'turtlebot_msgs/TakePanoramaResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'status';
info.MatPath{2} = 'STARTED';
info.MatPath{3} = 'IN_PROGRESS';
info.MatPath{4} = 'STOPPED';
