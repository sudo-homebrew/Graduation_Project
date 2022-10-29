function [data, info] = pR2GripperReleaseFeedback
%PR2GripperReleaseFeedback gives an empty data for pr2_gripper_sensor_msgs/PR2GripperReleaseFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperReleaseFeedback';
[data.Data, info.Data] = ros.internal.ros.messages.pr2_gripper_sensor_msgs.pR2GripperReleaseData;
info.Data.MLdataType = 'struct';
info.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperReleaseFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'data';
info.MatPath{2} = 'data.rtstate';
info.MatPath{3} = 'data.rtstate.realtime_controller_state';
info.MatPath{4} = 'data.rtstate.DISABLED';
info.MatPath{5} = 'data.rtstate.POSITION_SERVO';
info.MatPath{6} = 'data.rtstate.FORCE_SERVO';
info.MatPath{7} = 'data.rtstate.FIND_CONTACT';
info.MatPath{8} = 'data.rtstate.SLIP_SERVO';
