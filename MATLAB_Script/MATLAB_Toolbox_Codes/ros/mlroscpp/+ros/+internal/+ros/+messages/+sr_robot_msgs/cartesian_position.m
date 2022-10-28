function [data, info] = cartesian_position
%cartesian_position gives an empty data for sr_robot_msgs/cartesian_position

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/cartesian_position';
[data.TipName, info.TipName] = ros.internal.ros.messages.ros.char('string',0);
[data.TipPosX, info.TipPosX] = ros.internal.ros.messages.ros.default_type('single',1);
[data.TipPosY, info.TipPosY] = ros.internal.ros.messages.ros.default_type('single',1);
[data.TipPosZ, info.TipPosZ] = ros.internal.ros.messages.ros.default_type('single',1);
[data.TipOrientationRho, info.TipOrientationRho] = ros.internal.ros.messages.ros.default_type('single',1);
[data.TipOrientationTheta, info.TipOrientationTheta] = ros.internal.ros.messages.ros.default_type('single',1);
[data.TipOrientationSigma, info.TipOrientationSigma] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'sr_robot_msgs/cartesian_position';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'tip_name';
info.MatPath{2} = 'tip_pos_x';
info.MatPath{3} = 'tip_pos_y';
info.MatPath{4} = 'tip_pos_z';
info.MatPath{5} = 'tip_orientation_rho';
info.MatPath{6} = 'tip_orientation_theta';
info.MatPath{7} = 'tip_orientation_sigma';
