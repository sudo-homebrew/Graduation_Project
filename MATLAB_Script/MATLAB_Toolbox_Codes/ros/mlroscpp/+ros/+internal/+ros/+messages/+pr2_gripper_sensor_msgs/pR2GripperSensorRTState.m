function [data, info] = pR2GripperSensorRTState
%PR2GripperSensorRTState gives an empty data for pr2_gripper_sensor_msgs/PR2GripperSensorRTState

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperSensorRTState';
[data.RealtimeControllerState, info.RealtimeControllerState] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.DISABLED, info.DISABLED] = ros.internal.ros.messages.ros.default_type('int8',1, 0);
[data.POSITIONSERVO, info.POSITIONSERVO] = ros.internal.ros.messages.ros.default_type('int8',1, 3);
[data.FORCESERVO, info.FORCESERVO] = ros.internal.ros.messages.ros.default_type('int8',1, 4);
[data.FINDCONTACT, info.FINDCONTACT] = ros.internal.ros.messages.ros.default_type('int8',1, 5);
[data.SLIPSERVO, info.SLIPSERVO] = ros.internal.ros.messages.ros.default_type('int8',1, 6);
info.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperSensorRTState';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'realtime_controller_state';
info.MatPath{2} = 'DISABLED';
info.MatPath{3} = 'POSITION_SERVO';
info.MatPath{4} = 'FORCE_SERVO';
info.MatPath{5} = 'FIND_CONTACT';
info.MatPath{6} = 'SLIP_SERVO';
