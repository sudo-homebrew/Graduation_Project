function [data, info] = endEffectorProperties
%EndEffectorProperties gives an empty data for baxter_core_msgs/EndEffectorProperties

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'baxter_core_msgs/EndEffectorProperties';
[data.Id, info.Id] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.UiType, info.UiType] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.NOGRIPPER, info.NOGRIPPER] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.SUCTIONCUPGRIPPER, info.SUCTIONCUPGRIPPER] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.ELECTRICGRIPPER, info.ELECTRICGRIPPER] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.PASSIVEGRIPPER, info.PASSIVEGRIPPER] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.Manufacturer, info.Manufacturer] = ros.internal.ros.messages.ros.char('string',0);
[data.Product, info.Product] = ros.internal.ros.messages.ros.char('string',0);
[data.SerialNumber, info.SerialNumber] = ros.internal.ros.messages.ros.char('string',0);
[data.HardwareRev, info.HardwareRev] = ros.internal.ros.messages.ros.char('string',0);
[data.FirmwareRev, info.FirmwareRev] = ros.internal.ros.messages.ros.char('string',0);
[data.FirmwareDate, info.FirmwareDate] = ros.internal.ros.messages.ros.char('string',0);
[data.HasCalibration, info.HasCalibration] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.ControlsGrip, info.ControlsGrip] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.SensesGrip, info.SensesGrip] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.ReversesGrip, info.ReversesGrip] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.ControlsForce, info.ControlsForce] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.SensesForce, info.SensesForce] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.ControlsPosition, info.ControlsPosition] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.SensesPosition, info.SensesPosition] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Properties, info.Properties] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'baxter_core_msgs/EndEffectorProperties';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,21);
info.MatPath{1} = 'id';
info.MatPath{2} = 'ui_type';
info.MatPath{3} = 'NO_GRIPPER';
info.MatPath{4} = 'SUCTION_CUP_GRIPPER';
info.MatPath{5} = 'ELECTRIC_GRIPPER';
info.MatPath{6} = 'PASSIVE_GRIPPER';
info.MatPath{7} = 'manufacturer';
info.MatPath{8} = 'product';
info.MatPath{9} = 'serial_number';
info.MatPath{10} = 'hardware_rev';
info.MatPath{11} = 'firmware_rev';
info.MatPath{12} = 'firmware_date';
info.MatPath{13} = 'has_calibration';
info.MatPath{14} = 'controls_grip';
info.MatPath{15} = 'senses_grip';
info.MatPath{16} = 'reverses_grip';
info.MatPath{17} = 'controls_force';
info.MatPath{18} = 'senses_force';
info.MatPath{19} = 'controls_position';
info.MatPath{20} = 'senses_position';
info.MatPath{21} = 'properties';