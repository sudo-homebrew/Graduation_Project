function [data, info] = fC2OCS
%FC2OCS gives an empty data for jsk_network_tools/FC2OCS

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_network_tools/FC2OCS';
[data.JointAngles, info.JointAngles] = ros.internal.ros.messages.ros.default_type('uint8',32);
[data.LhandForce, info.LhandForce] = ros.internal.ros.messages.ros.default_type('uint8',6);
[data.RhandForce, info.RhandForce] = ros.internal.ros.messages.ros.default_type('uint8',6);
[data.LfootForce, info.LfootForce] = ros.internal.ros.messages.ros.default_type('uint8',6);
[data.RfootForce, info.RfootForce] = ros.internal.ros.messages.ros.default_type('uint8',6);
[data.ServoState, info.ServoState] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'jsk_network_tools/FC2OCS';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'joint_angles';
info.MatPath{2} = 'lhand_force';
info.MatPath{3} = 'rhand_force';
info.MatPath{4} = 'lfoot_force';
info.MatPath{5} = 'rfoot_force';
info.MatPath{6} = 'servo_state';
