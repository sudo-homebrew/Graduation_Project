function [data, info] = pR2GripperEventDetectorData
%PR2GripperEventDetectorData gives an empty data for pr2_gripper_sensor_msgs/PR2GripperEventDetectorData

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperEventDetectorData';
[data.Stamp, info.Stamp] = ros.internal.ros.messages.ros.time;
info.Stamp.MLdataType = 'struct';
[data.TriggerConditionsMet, info.TriggerConditionsMet] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.SlipEvent, info.SlipEvent] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.AccelerationEvent, info.AccelerationEvent] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.AccelerationVector, info.AccelerationVector] = ros.internal.ros.messages.ros.default_type('double',3);
info.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperEventDetectorData';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'stamp';
info.MatPath{2} = 'stamp.sec';
info.MatPath{3} = 'stamp.nsec';
info.MatPath{4} = 'trigger_conditions_met';
info.MatPath{5} = 'slip_event';
info.MatPath{6} = 'acceleration_event';
info.MatPath{7} = 'acceleration_vector';
