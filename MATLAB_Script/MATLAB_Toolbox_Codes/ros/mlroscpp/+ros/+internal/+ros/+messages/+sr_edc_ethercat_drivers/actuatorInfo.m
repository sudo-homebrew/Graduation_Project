function [data, info] = actuatorInfo
%ActuatorInfo gives an empty data for sr_edc_ethercat_drivers/ActuatorInfo

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_edc_ethercat_drivers/ActuatorInfo';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.SlowMotorCurrentLimit, info.SlowMotorCurrentLimit] = ros.internal.ros.messages.ros.default_type('double',1);
[data.QuickMotorCurrentLimit, info.QuickMotorCurrentLimit] = ros.internal.ros.messages.ros.default_type('double',1);
[data.DutyLimit, info.DutyLimit] = ros.internal.ros.messages.ros.default_type('double',1);
[data.MaxDuty, info.MaxDuty] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'sr_edc_ethercat_drivers/ActuatorInfo';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'name';
info.MatPath{2} = 'slow_motor_current_limit';
info.MatPath{3} = 'quick_motor_current_limit';
info.MatPath{4} = 'duty_limit';
info.MatPath{5} = 'max_duty';
