function [data, info] = pR2GripperSlipServoData
%PR2GripperSlipServoData gives an empty data for pr2_gripper_sensor_msgs/PR2GripperSlipServoData

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperSlipServoData';
[data.Stamp, info.Stamp] = ros.internal.ros.messages.ros.time;
info.Stamp.MLdataType = 'struct';
[data.Deformation, info.Deformation] = ros.internal.ros.messages.ros.default_type('double',1);
[data.LeftFingertipPadForce, info.LeftFingertipPadForce] = ros.internal.ros.messages.ros.default_type('double',1);
[data.RightFingertipPadForce, info.RightFingertipPadForce] = ros.internal.ros.messages.ros.default_type('double',1);
[data.JointEffort, info.JointEffort] = ros.internal.ros.messages.ros.default_type('double',1);
[data.SlipDetected, info.SlipDetected] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.DeformationLimitReached, info.DeformationLimitReached] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.FingertipForceLimitReached, info.FingertipForceLimitReached] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.GripperEmpty, info.GripperEmpty] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Rtstate, info.Rtstate] = ros.internal.ros.messages.pr2_gripper_sensor_msgs.pR2GripperSensorRTState;
info.Rtstate.MLdataType = 'struct';
info.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperSlipServoData';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,18);
info.MatPath{1} = 'stamp';
info.MatPath{2} = 'stamp.sec';
info.MatPath{3} = 'stamp.nsec';
info.MatPath{4} = 'deformation';
info.MatPath{5} = 'left_fingertip_pad_force';
info.MatPath{6} = 'right_fingertip_pad_force';
info.MatPath{7} = 'joint_effort';
info.MatPath{8} = 'slip_detected';
info.MatPath{9} = 'deformation_limit_reached';
info.MatPath{10} = 'fingertip_force_limit_reached';
info.MatPath{11} = 'gripper_empty';
info.MatPath{12} = 'rtstate';
info.MatPath{13} = 'rtstate.realtime_controller_state';
info.MatPath{14} = 'rtstate.DISABLED';
info.MatPath{15} = 'rtstate.POSITION_SERVO';
info.MatPath{16} = 'rtstate.FORCE_SERVO';
info.MatPath{17} = 'rtstate.FIND_CONTACT';
info.MatPath{18} = 'rtstate.SLIP_SERVO';
