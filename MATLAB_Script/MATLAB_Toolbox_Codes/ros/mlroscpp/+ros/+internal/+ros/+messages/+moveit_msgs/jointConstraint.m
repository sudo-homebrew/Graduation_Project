function [data, info] = jointConstraint
%JointConstraint gives an empty data for moveit_msgs/JointConstraint

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/JointConstraint';
[data.JointName, info.JointName] = ros.internal.ros.messages.ros.char('string',0);
[data.Position, info.Position] = ros.internal.ros.messages.ros.default_type('double',1);
[data.ToleranceAbove, info.ToleranceAbove] = ros.internal.ros.messages.ros.default_type('double',1);
[data.ToleranceBelow, info.ToleranceBelow] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Weight, info.Weight] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'moveit_msgs/JointConstraint';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'joint_name';
info.MatPath{2} = 'position';
info.MatPath{3} = 'tolerance_above';
info.MatPath{4} = 'tolerance_below';
info.MatPath{5} = 'weight';
