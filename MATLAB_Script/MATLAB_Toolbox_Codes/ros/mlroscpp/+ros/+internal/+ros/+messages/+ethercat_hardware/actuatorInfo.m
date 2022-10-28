function [data, info] = actuatorInfo
%ActuatorInfo gives an empty data for ethercat_hardware/ActuatorInfo

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ethercat_hardware/ActuatorInfo';
[data.Id, info.Id] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.RobotName, info.RobotName] = ros.internal.ros.messages.ros.char('string',0);
[data.MotorMake, info.MotorMake] = ros.internal.ros.messages.ros.char('string',0);
[data.MotorModel, info.MotorModel] = ros.internal.ros.messages.ros.char('string',0);
[data.MaxCurrent, info.MaxCurrent] = ros.internal.ros.messages.ros.default_type('double',1);
[data.SpeedConstant, info.SpeedConstant] = ros.internal.ros.messages.ros.default_type('double',1);
[data.MotorResistance, info.MotorResistance] = ros.internal.ros.messages.ros.default_type('double',1);
[data.MotorTorqueConstant, info.MotorTorqueConstant] = ros.internal.ros.messages.ros.default_type('double',1);
[data.EncoderReduction, info.EncoderReduction] = ros.internal.ros.messages.ros.default_type('double',1);
[data.PulsesPerRevolution, info.PulsesPerRevolution] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'ethercat_hardware/ActuatorInfo';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'id';
info.MatPath{2} = 'name';
info.MatPath{3} = 'robot_name';
info.MatPath{4} = 'motor_make';
info.MatPath{5} = 'motor_model';
info.MatPath{6} = 'max_current';
info.MatPath{7} = 'speed_constant';
info.MatPath{8} = 'motor_resistance';
info.MatPath{9} = 'motor_torque_constant';
info.MatPath{10} = 'encoder_reduction';
info.MatPath{11} = 'pulses_per_revolution';
