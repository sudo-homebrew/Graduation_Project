function [data, info] = pR2GripperGrabData
%PR2GripperGrabData gives an empty data for pr2_gripper_sensor_msgs/PR2GripperGrabData

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperGrabData';
[data.Rtstate, info.Rtstate] = ros.internal.ros.messages.pr2_gripper_sensor_msgs.pR2GripperSensorRTState;
info.Rtstate.MLdataType = 'struct';
info.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperGrabData';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'rtstate';
info.MatPath{2} = 'rtstate.realtime_controller_state';
info.MatPath{3} = 'rtstate.DISABLED';
info.MatPath{4} = 'rtstate.POSITION_SERVO';
info.MatPath{5} = 'rtstate.FORCE_SERVO';
info.MatPath{6} = 'rtstate.FIND_CONTACT';
info.MatPath{7} = 'rtstate.SLIP_SERVO';
