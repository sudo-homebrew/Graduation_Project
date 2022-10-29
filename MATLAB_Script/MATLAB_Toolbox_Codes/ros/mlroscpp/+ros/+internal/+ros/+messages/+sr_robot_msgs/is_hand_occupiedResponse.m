function [data, info] = is_hand_occupiedResponse
%is_hand_occupied gives an empty data for sr_robot_msgs/is_hand_occupiedResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/is_hand_occupiedResponse';
[data.HandOccupied, info.HandOccupied] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'sr_robot_msgs/is_hand_occupiedResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'hand_occupied';
