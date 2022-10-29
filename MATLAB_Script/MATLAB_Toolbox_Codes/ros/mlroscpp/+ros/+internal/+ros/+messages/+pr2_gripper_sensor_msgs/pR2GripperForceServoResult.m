function [data, info] = pR2GripperForceServoResult
%PR2GripperForceServoResult gives an empty data for pr2_gripper_sensor_msgs/PR2GripperForceServoResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperForceServoResult';
[data.Data, info.Data] = ros.internal.ros.messages.pr2_gripper_sensor_msgs.pR2GripperForceServoData;
info.Data.MLdataType = 'struct';
info.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperForceServoResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,15);
info.MatPath{1} = 'data';
info.MatPath{2} = 'data.stamp';
info.MatPath{3} = 'data.stamp.sec';
info.MatPath{4} = 'data.stamp.nsec';
info.MatPath{5} = 'data.left_fingertip_pad_force';
info.MatPath{6} = 'data.right_fingertip_pad_force';
info.MatPath{7} = 'data.joint_effort';
info.MatPath{8} = 'data.force_achieved';
info.MatPath{9} = 'data.rtstate';
info.MatPath{10} = 'data.rtstate.realtime_controller_state';
info.MatPath{11} = 'data.rtstate.DISABLED';
info.MatPath{12} = 'data.rtstate.POSITION_SERVO';
info.MatPath{13} = 'data.rtstate.FORCE_SERVO';
info.MatPath{14} = 'data.rtstate.FIND_CONTACT';
info.MatPath{15} = 'data.rtstate.SLIP_SERVO';
