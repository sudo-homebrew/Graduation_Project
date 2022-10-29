function [data, info] = boardInfo
%BoardInfo gives an empty data for ethercat_hardware/BoardInfo

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ethercat_hardware/BoardInfo';
[data.Description, info.Description] = ros.internal.ros.messages.ros.char('string',0);
[data.ProductCode, info.ProductCode] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Pcb, info.Pcb] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Pca, info.Pca] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Serial, info.Serial] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.FirmwareMajor, info.FirmwareMajor] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.FirmwareMinor, info.FirmwareMinor] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.BoardResistance, info.BoardResistance] = ros.internal.ros.messages.ros.default_type('double',1);
[data.MaxPwmRatio, info.MaxPwmRatio] = ros.internal.ros.messages.ros.default_type('double',1);
[data.HwMaxCurrent, info.HwMaxCurrent] = ros.internal.ros.messages.ros.default_type('double',1);
[data.PoorMeasuredMotorVoltage, info.PoorMeasuredMotorVoltage] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'ethercat_hardware/BoardInfo';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'description';
info.MatPath{2} = 'product_code';
info.MatPath{3} = 'pcb';
info.MatPath{4} = 'pca';
info.MatPath{5} = 'serial';
info.MatPath{6} = 'firmware_major';
info.MatPath{7} = 'firmware_minor';
info.MatPath{8} = 'board_resistance';
info.MatPath{9} = 'max_pwm_ratio';
info.MatPath{10} = 'hw_max_current';
info.MatPath{11} = 'poor_measured_motor_voltage';
