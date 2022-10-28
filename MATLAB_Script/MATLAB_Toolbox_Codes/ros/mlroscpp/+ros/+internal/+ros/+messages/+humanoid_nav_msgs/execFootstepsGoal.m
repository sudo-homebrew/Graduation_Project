function [data, info] = execFootstepsGoal
%ExecFootstepsGoal gives an empty data for humanoid_nav_msgs/ExecFootstepsGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'humanoid_nav_msgs/ExecFootstepsGoal';
[data.Footsteps, info.Footsteps] = ros.internal.ros.messages.humanoid_nav_msgs.stepTarget;
info.Footsteps.MLdataType = 'struct';
info.Footsteps.MaxLen = NaN;
info.Footsteps.MinLen = 0;
data.Footsteps = data.Footsteps([],1);
[data.FeedbackFrequency, info.FeedbackFrequency] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'humanoid_nav_msgs/ExecFootstepsGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'footsteps';
info.MatPath{2} = 'footsteps.pose';
info.MatPath{3} = 'footsteps.pose.x';
info.MatPath{4} = 'footsteps.pose.y';
info.MatPath{5} = 'footsteps.pose.theta';
info.MatPath{6} = 'footsteps.leg';
info.MatPath{7} = 'footsteps.right';
info.MatPath{8} = 'footsteps.left';
info.MatPath{9} = 'feedback_frequency';
