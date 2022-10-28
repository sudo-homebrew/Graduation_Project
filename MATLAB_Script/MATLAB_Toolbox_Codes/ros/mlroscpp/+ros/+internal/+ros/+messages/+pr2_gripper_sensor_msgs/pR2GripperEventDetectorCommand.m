function [data, info] = pR2GripperEventDetectorCommand
%PR2GripperEventDetectorCommand gives an empty data for pr2_gripper_sensor_msgs/PR2GripperEventDetectorCommand

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperEventDetectorCommand';
[data.TriggerConditions, info.TriggerConditions] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.FINGERSIDEIMPACTORACC, info.FINGERSIDEIMPACTORACC] = ros.internal.ros.messages.ros.default_type('int8',1, 0);
[data.SLIPANDACC, info.SLIPANDACC] = ros.internal.ros.messages.ros.default_type('int8',1, 1);
[data.FINGERSIDEIMPACTORSLIPORACC, info.FINGERSIDEIMPACTORSLIPORACC] = ros.internal.ros.messages.ros.default_type('int8',1, 2);
[data.SLIP, info.SLIP] = ros.internal.ros.messages.ros.default_type('int8',1, 3);
[data.ACC, info.ACC] = ros.internal.ros.messages.ros.default_type('int8',1, 4);
[data.AccelerationTriggerMagnitude, info.AccelerationTriggerMagnitude] = ros.internal.ros.messages.ros.default_type('double',1);
[data.SlipTriggerMagnitude, info.SlipTriggerMagnitude] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperEventDetectorCommand';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'trigger_conditions';
info.MatPath{2} = 'FINGER_SIDE_IMPACT_OR_ACC';
info.MatPath{3} = 'SLIP_AND_ACC';
info.MatPath{4} = 'FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC';
info.MatPath{5} = 'SLIP';
info.MatPath{6} = 'ACC';
info.MatPath{7} = 'acceleration_trigger_magnitude';
info.MatPath{8} = 'slip_trigger_magnitude';
