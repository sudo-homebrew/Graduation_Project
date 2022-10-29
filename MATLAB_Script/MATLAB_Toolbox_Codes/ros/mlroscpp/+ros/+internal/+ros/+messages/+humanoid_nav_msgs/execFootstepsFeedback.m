function [data, info] = execFootstepsFeedback
%ExecFootstepsFeedback gives an empty data for humanoid_nav_msgs/ExecFootstepsFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'humanoid_nav_msgs/ExecFootstepsFeedback';
[data.ExecutedFootsteps, info.ExecutedFootsteps] = ros.internal.ros.messages.humanoid_nav_msgs.stepTarget;
info.ExecutedFootsteps.MLdataType = 'struct';
info.ExecutedFootsteps.MaxLen = NaN;
info.ExecutedFootsteps.MinLen = 0;
data.ExecutedFootsteps = data.ExecutedFootsteps([],1);
info.MessageType = 'humanoid_nav_msgs/ExecFootstepsFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'executed_footsteps';
info.MatPath{2} = 'executed_footsteps.pose';
info.MatPath{3} = 'executed_footsteps.pose.x';
info.MatPath{4} = 'executed_footsteps.pose.y';
info.MatPath{5} = 'executed_footsteps.pose.theta';
info.MatPath{6} = 'executed_footsteps.leg';
info.MatPath{7} = 'executed_footsteps.right';
info.MatPath{8} = 'executed_footsteps.left';
