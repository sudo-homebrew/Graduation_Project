function [data, info] = cartesian_data
%cartesian_data gives an empty data for sr_robot_msgs/cartesian_data

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/cartesian_data';
[data.CartesianPositionsLength, info.CartesianPositionsLength] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.CartesianPositions, info.CartesianPositions] = ros.internal.ros.messages.sr_robot_msgs.cartesian_position;
info.CartesianPositions.MLdataType = 'struct';
info.CartesianPositions.MaxLen = NaN;
info.CartesianPositions.MinLen = 0;
data.CartesianPositions = data.CartesianPositions([],1);
info.MessageType = 'sr_robot_msgs/cartesian_data';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'cartesian_positions_length';
info.MatPath{2} = 'cartesian_positions';
info.MatPath{3} = 'cartesian_positions.tip_name';
info.MatPath{4} = 'cartesian_positions.tip_pos_x';
info.MatPath{5} = 'cartesian_positions.tip_pos_y';
info.MatPath{6} = 'cartesian_positions.tip_pos_z';
info.MatPath{7} = 'cartesian_positions.tip_orientation_rho';
info.MatPath{8} = 'cartesian_positions.tip_orientation_theta';
info.MatPath{9} = 'cartesian_positions.tip_orientation_sigma';
