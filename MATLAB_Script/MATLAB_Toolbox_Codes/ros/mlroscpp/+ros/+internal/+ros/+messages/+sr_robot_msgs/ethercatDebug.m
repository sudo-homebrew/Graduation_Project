function [data, info] = ethercatDebug
%EthercatDebug gives an empty data for sr_robot_msgs/EthercatDebug

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/EthercatDebug';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Sensors, info.Sensors] = ros.internal.ros.messages.ros.default_type('int16',NaN);
[data.MotorDataType, info.MotorDataType] = ros.internal.ros.messages.sr_robot_msgs.fromMotorDataType;
info.MotorDataType.MLdataType = 'struct';
[data.WhichMotors, info.WhichMotors] = ros.internal.ros.messages.ros.default_type('int16',1);
[data.WhichMotorDataArrived, info.WhichMotorDataArrived] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.WhichMotorDataHadErrors, info.WhichMotorDataHadErrors] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.MotorDataPacketTorque, info.MotorDataPacketTorque] = ros.internal.ros.messages.ros.default_type('int16',NaN);
[data.MotorDataPacketMisc, info.MotorDataPacketMisc] = ros.internal.ros.messages.ros.default_type('int16',NaN);
[data.TactileDataType, info.TactileDataType] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.TactileDataValid, info.TactileDataValid] = ros.internal.ros.messages.ros.default_type('int16',1);
[data.Tactile, info.Tactile] = ros.internal.ros.messages.ros.default_type('int16',NaN);
[data.IdleTimeUs, info.IdleTimeUs] = ros.internal.ros.messages.ros.default_type('int16',1);
info.MessageType = 'sr_robot_msgs/EthercatDebug';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,18);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'sensors';
info.MatPath{8} = 'motor_data_type';
info.MatPath{9} = 'motor_data_type.data';
info.MatPath{10} = 'which_motors';
info.MatPath{11} = 'which_motor_data_arrived';
info.MatPath{12} = 'which_motor_data_had_errors';
info.MatPath{13} = 'motor_data_packet_torque';
info.MatPath{14} = 'motor_data_packet_misc';
info.MatPath{15} = 'tactile_data_type';
info.MatPath{16} = 'tactile_data_valid';
info.MatPath{17} = 'tactile';
info.MatPath{18} = 'idle_time_us';