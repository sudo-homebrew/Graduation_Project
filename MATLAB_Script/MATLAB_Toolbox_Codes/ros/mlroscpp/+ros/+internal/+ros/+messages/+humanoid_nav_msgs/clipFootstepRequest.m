function [data, info] = clipFootstepRequest
%ClipFootstep gives an empty data for humanoid_nav_msgs/ClipFootstepRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'humanoid_nav_msgs/ClipFootstepRequest';
[data.Step, info.Step] = ros.internal.ros.messages.humanoid_nav_msgs.stepTarget;
info.Step.MLdataType = 'struct';
info.MessageType = 'humanoid_nav_msgs/ClipFootstepRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'step';
info.MatPath{2} = 'step.pose';
info.MatPath{3} = 'step.pose.x';
info.MatPath{4} = 'step.pose.y';
info.MatPath{5} = 'step.pose.theta';
info.MatPath{6} = 'step.leg';
info.MatPath{7} = 'step.right';
info.MatPath{8} = 'step.left';
