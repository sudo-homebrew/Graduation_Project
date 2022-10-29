function [data, info] = gripperTranslation
%GripperTranslation gives an empty data for manipulation_msgs/GripperTranslation

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'manipulation_msgs/GripperTranslation';
[data.Direction, info.Direction] = ros.internal.ros.messages.geometry_msgs.vector3Stamped;
info.Direction.MLdataType = 'struct';
[data.DesiredDistance, info.DesiredDistance] = ros.internal.ros.messages.ros.default_type('single',1);
[data.MinDistance, info.MinDistance] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'manipulation_msgs/GripperTranslation';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,13);
info.MatPath{1} = 'direction';
info.MatPath{2} = 'direction.header';
info.MatPath{3} = 'direction.header.seq';
info.MatPath{4} = 'direction.header.stamp';
info.MatPath{5} = 'direction.header.stamp.sec';
info.MatPath{6} = 'direction.header.stamp.nsec';
info.MatPath{7} = 'direction.header.frame_id';
info.MatPath{8} = 'direction.vector';
info.MatPath{9} = 'direction.vector.x';
info.MatPath{10} = 'direction.vector.y';
info.MatPath{11} = 'direction.vector.z';
info.MatPath{12} = 'desired_distance';
info.MatPath{13} = 'min_distance';
