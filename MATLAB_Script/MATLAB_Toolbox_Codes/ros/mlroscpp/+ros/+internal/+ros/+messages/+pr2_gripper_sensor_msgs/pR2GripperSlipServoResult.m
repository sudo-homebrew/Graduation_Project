function [data, info] = pR2GripperSlipServoResult
%PR2GripperSlipServoResult gives an empty data for pr2_gripper_sensor_msgs/PR2GripperSlipServoResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperSlipServoResult';
[data.Data, info.Data] = ros.internal.ros.messages.pr2_gripper_sensor_msgs.pR2GripperSlipServoData;
info.Data.MLdataType = 'struct';
info.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperSlipServoResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,19);
info.MatPath{1} = 'data';
info.MatPath{2} = 'data.stamp';
info.MatPath{3} = 'data.stamp.sec';
info.MatPath{4} = 'data.stamp.nsec';
info.MatPath{5} = 'data.deformation';
info.MatPath{6} = 'data.left_fingertip_pad_force';
info.MatPath{7} = 'data.right_fingertip_pad_force';
info.MatPath{8} = 'data.joint_effort';
info.MatPath{9} = 'data.slip_detected';
info.MatPath{10} = 'data.deformation_limit_reached';
info.MatPath{11} = 'data.fingertip_force_limit_reached';
info.MatPath{12} = 'data.gripper_empty';
info.MatPath{13} = 'data.rtstate';
info.MatPath{14} = 'data.rtstate.realtime_controller_state';
info.MatPath{15} = 'data.rtstate.DISABLED';
info.MatPath{16} = 'data.rtstate.POSITION_SERVO';
info.MatPath{17} = 'data.rtstate.FORCE_SERVO';
info.MatPath{18} = 'data.rtstate.FIND_CONTACT';
info.MatPath{19} = 'data.rtstate.SLIP_SERVO';
