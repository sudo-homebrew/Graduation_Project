function [data, info] = pose
%Pose gives an empty data for geometry_msgs/Pose

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/Pose';
[data.Position, info.Position] = ros.internal.ros.messages.geometry_msgs.point;
info.Position.MLdataType = 'struct';
[data.Orientation, info.Orientation] = ros.internal.ros.messages.geometry_msgs.quaternion;
info.Orientation.MLdataType = 'struct';
info.MessageType = 'geometry_msgs/Pose';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'position';
info.MatPath{2} = 'position.x';
info.MatPath{3} = 'position.y';
info.MatPath{4} = 'position.z';
info.MatPath{5} = 'orientation';
info.MatPath{6} = 'orientation.x';
info.MatPath{7} = 'orientation.y';
info.MatPath{8} = 'orientation.z';
info.MatPath{9} = 'orientation.w';
