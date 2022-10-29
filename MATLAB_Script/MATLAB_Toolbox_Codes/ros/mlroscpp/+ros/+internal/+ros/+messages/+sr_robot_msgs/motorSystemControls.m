function [data, info] = motorSystemControls
%MotorSystemControls gives an empty data for sr_robot_msgs/MotorSystemControls

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/MotorSystemControls';
[data.MotorId, info.MotorId] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.EnableBacklashCompensation, info.EnableBacklashCompensation] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.IncreaseSglTracking, info.IncreaseSglTracking] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.DecreaseSglTracking, info.DecreaseSglTracking] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.IncreaseSgrTracking, info.IncreaseSgrTracking] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.DecreaseSgrTracking, info.DecreaseSgrTracking] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.InitiateJiggling, info.InitiateJiggling] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.WriteConfigToEeprom, info.WriteConfigToEeprom] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'sr_robot_msgs/MotorSystemControls';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'motor_id';
info.MatPath{2} = 'enable_backlash_compensation';
info.MatPath{3} = 'increase_sgl_tracking';
info.MatPath{4} = 'decrease_sgl_tracking';
info.MatPath{5} = 'increase_sgr_tracking';
info.MatPath{6} = 'decrease_sgr_tracking';
info.MatPath{7} = 'initiate_jiggling';
info.MatPath{8} = 'write_config_to_eeprom';
