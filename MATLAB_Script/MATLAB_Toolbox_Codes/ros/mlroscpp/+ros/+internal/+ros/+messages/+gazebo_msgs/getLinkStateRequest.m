function [data, info] = getLinkStateRequest
%GetLinkState gives an empty data for gazebo_msgs/GetLinkStateRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gazebo_msgs/GetLinkStateRequest';
[data.LinkName, info.LinkName] = ros.internal.ros.messages.ros.char('string',0);
[data.ReferenceFrame, info.ReferenceFrame] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'gazebo_msgs/GetLinkStateRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'link_name';
info.MatPath{2} = 'reference_frame';
