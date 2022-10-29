function [data, info] = pR2GripperForceServoData
%PR2GripperForceServoData gives an empty data for pr2_gripper_sensor_msgs/PR2GripperForceServoData

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperForceServoData';
[data.Stamp, info.Stamp] = ros.internal.ros.messages.ros.time;
info.Stamp.MLdataType = 'struct';
[data.LeftFingertipPadForce, info.LeftFingertipPadForce] = ros.internal.ros.messages.ros.default_type('double',1);
[data.RightFingertipPadForce, info.RightFingertipPadForce] = ros.internal.ros.messages.ros.default_type('double',1);
[data.JointEffort, info.JointEffort] = ros.internal.ros.messages.ros.default_type('double',1);
[data.ForceAchieved, info.ForceAchieved] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Rtstate, info.Rtstate] = ros.internal.ros.messages.pr2_gripper_sensor_msgs.pR2GripperSensorRTState;
info.Rtstate.MLdataType = 'struct';
info.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperForceServoData';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'stamp';
info.MatPath{2} = 'stamp.sec';
info.MatPath{3} = 'stamp.nsec';
info.MatPath{4} = 'left_fingertip_pad_force';
info.MatPath{5} = 'right_fingertip_pad_force';
info.MatPath{6} = 'joint_effort';
info.MatPath{7} = 'force_achieved';
info.MatPath{8} = 'rtstate';
info.MatPath{9} = 'rtstate.realtime_controller_state';
info.MatPath{10} = 'rtstate.DISABLED';
info.MatPath{11} = 'rtstate.POSITION_SERVO';
info.MatPath{12} = 'rtstate.FORCE_SERVO';
info.MatPath{13} = 'rtstate.FIND_CONTACT';
info.MatPath{14} = 'rtstate.SLIP_SERVO';
