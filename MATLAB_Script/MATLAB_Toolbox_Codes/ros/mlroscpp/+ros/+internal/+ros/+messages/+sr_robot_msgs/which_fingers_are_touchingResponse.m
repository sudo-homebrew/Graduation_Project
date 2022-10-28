function [data, info] = which_fingers_are_touchingResponse
%which_fingers_are_touching gives an empty data for sr_robot_msgs/which_fingers_are_touchingResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/which_fingers_are_touchingResponse';
[data.TouchForces, info.TouchForces] = ros.internal.ros.messages.ros.default_type('double',NaN);
info.MessageType = 'sr_robot_msgs/which_fingers_are_touchingResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'touch_forces';
