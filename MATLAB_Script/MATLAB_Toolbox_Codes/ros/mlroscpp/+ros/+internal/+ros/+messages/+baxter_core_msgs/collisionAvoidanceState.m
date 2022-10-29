function [data, info] = collisionAvoidanceState
%CollisionAvoidanceState gives an empty data for baxter_core_msgs/CollisionAvoidanceState

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'baxter_core_msgs/CollisionAvoidanceState';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.OtherArm, info.OtherArm] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.CollisionObject, info.CollisionObject] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'baxter_core_msgs/CollisionAvoidanceState';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'other_arm';
info.MatPath{8} = 'collision_object';
