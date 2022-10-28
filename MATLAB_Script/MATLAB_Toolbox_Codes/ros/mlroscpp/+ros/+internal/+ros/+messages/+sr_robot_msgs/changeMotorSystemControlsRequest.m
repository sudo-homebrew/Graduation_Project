function [data, info] = changeMotorSystemControlsRequest
%ChangeMotorSystemControls gives an empty data for sr_robot_msgs/ChangeMotorSystemControlsRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/ChangeMotorSystemControlsRequest';
[data.MotorSystemControls, info.MotorSystemControls] = ros.internal.ros.messages.sr_robot_msgs.motorSystemControls;
info.MotorSystemControls.MLdataType = 'struct';
info.MotorSystemControls.MaxLen = NaN;
info.MotorSystemControls.MinLen = 0;
data.MotorSystemControls = data.MotorSystemControls([],1);
info.MessageType = 'sr_robot_msgs/ChangeMotorSystemControlsRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'motor_system_controls';
info.MatPath{2} = 'motor_system_controls.motor_id';
info.MatPath{3} = 'motor_system_controls.enable_backlash_compensation';
info.MatPath{4} = 'motor_system_controls.increase_sgl_tracking';
info.MatPath{5} = 'motor_system_controls.decrease_sgl_tracking';
info.MatPath{6} = 'motor_system_controls.increase_sgr_tracking';
info.MatPath{7} = 'motor_system_controls.decrease_sgr_tracking';
info.MatPath{8} = 'motor_system_controls.initiate_jiggling';
info.MatPath{9} = 'motor_system_controls.write_config_to_eeprom';
