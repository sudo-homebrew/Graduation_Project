function [data, info] = wrench
%Wrench gives an empty data for geometry_msgs/Wrench

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/Wrench';
[data.Force, info.Force] = ros.internal.ros.messages.geometry_msgs.vector3;
info.Force.MLdataType = 'struct';
[data.Torque, info.Torque] = ros.internal.ros.messages.geometry_msgs.vector3;
info.Torque.MLdataType = 'struct';
info.MessageType = 'geometry_msgs/Wrench';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'force';
info.MatPath{2} = 'force.x';
info.MatPath{3} = 'force.y';
info.MatPath{4} = 'force.z';
info.MatPath{5} = 'torque';
info.MatPath{6} = 'torque.x';
info.MatPath{7} = 'torque.y';
info.MatPath{8} = 'torque.z';
