function [data, info] = getJointStateResponse
%getJointState gives an empty data for sr_utilities/getJointStateResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_utilities/getJointStateResponse';
[data.JointState, info.JointState] = ros.internal.ros.messages.sensor_msgs.jointState;
info.JointState.MLdataType = 'struct';
info.MessageType = 'sr_utilities/getJointStateResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'joint_state';
info.MatPath{2} = 'joint_state.header';
info.MatPath{3} = 'joint_state.header.seq';
info.MatPath{4} = 'joint_state.header.stamp';
info.MatPath{5} = 'joint_state.header.stamp.sec';
info.MatPath{6} = 'joint_state.header.stamp.nsec';
info.MatPath{7} = 'joint_state.header.frame_id';
info.MatPath{8} = 'joint_state.name';
info.MatPath{9} = 'joint_state.position';
info.MatPath{10} = 'joint_state.velocity';
info.MatPath{11} = 'joint_state.effort';
