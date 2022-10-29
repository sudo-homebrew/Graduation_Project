function [data, info] = motorStateList
%MotorStateList gives an empty data for dynamixel_msgs/MotorStateList

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'dynamixel_msgs/MotorStateList';
[data.MotorStates, info.MotorStates] = ros.internal.ros.messages.dynamixel_msgs.motorState;
info.MotorStates.MLdataType = 'struct';
info.MotorStates.MaxLen = NaN;
info.MotorStates.MinLen = 0;
data.MotorStates = data.MotorStates([],1);
info.MessageType = 'dynamixel_msgs/MotorStateList';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'motor_states';
info.MatPath{2} = 'motor_states.timestamp';
info.MatPath{3} = 'motor_states.id';
info.MatPath{4} = 'motor_states.goal';
info.MatPath{5} = 'motor_states.position';
info.MatPath{6} = 'motor_states.error';
info.MatPath{7} = 'motor_states.speed';
info.MatPath{8} = 'motor_states.load';
info.MatPath{9} = 'motor_states.voltage';
info.MatPath{10} = 'motor_states.temperature';
info.MatPath{11} = 'motor_states.moving';
