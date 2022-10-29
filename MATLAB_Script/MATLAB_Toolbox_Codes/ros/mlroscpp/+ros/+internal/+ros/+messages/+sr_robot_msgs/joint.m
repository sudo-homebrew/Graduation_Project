function [data, info] = joint
%joint gives an empty data for sr_robot_msgs/joint

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/joint';
[data.JointName, info.JointName] = ros.internal.ros.messages.ros.char('string',0);
[data.JointPosition, info.JointPosition] = ros.internal.ros.messages.ros.default_type('double',1);
[data.JointTarget, info.JointTarget] = ros.internal.ros.messages.ros.default_type('double',1);
[data.JointTorque, info.JointTorque] = ros.internal.ros.messages.ros.default_type('double',1);
[data.JointTemperature, info.JointTemperature] = ros.internal.ros.messages.ros.default_type('double',1);
[data.JointCurrent, info.JointCurrent] = ros.internal.ros.messages.ros.default_type('double',1);
[data.ErrorFlag, info.ErrorFlag] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'sr_robot_msgs/joint';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'joint_name';
info.MatPath{2} = 'joint_position';
info.MatPath{3} = 'joint_target';
info.MatPath{4} = 'joint_torque';
info.MatPath{5} = 'joint_temperature';
info.MatPath{6} = 'joint_current';
info.MatPath{7} = 'error_flag';
