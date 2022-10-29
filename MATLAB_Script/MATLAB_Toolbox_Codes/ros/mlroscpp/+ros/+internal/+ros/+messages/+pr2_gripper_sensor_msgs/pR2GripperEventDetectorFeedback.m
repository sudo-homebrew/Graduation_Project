function [data, info] = pR2GripperEventDetectorFeedback
%PR2GripperEventDetectorFeedback gives an empty data for pr2_gripper_sensor_msgs/PR2GripperEventDetectorFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperEventDetectorFeedback';
[data.Data, info.Data] = ros.internal.ros.messages.pr2_gripper_sensor_msgs.pR2GripperEventDetectorData;
info.Data.MLdataType = 'struct';
info.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperEventDetectorFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'data';
info.MatPath{2} = 'data.stamp';
info.MatPath{3} = 'data.stamp.sec';
info.MatPath{4} = 'data.stamp.nsec';
info.MatPath{5} = 'data.trigger_conditions_met';
info.MatPath{6} = 'data.slip_event';
info.MatPath{7} = 'data.acceleration_event';
info.MatPath{8} = 'data.acceleration_vector';
