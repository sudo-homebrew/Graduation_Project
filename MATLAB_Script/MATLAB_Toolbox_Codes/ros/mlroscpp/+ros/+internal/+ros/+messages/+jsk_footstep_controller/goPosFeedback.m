function [data, info] = goPosFeedback
%GoPosFeedback gives an empty data for jsk_footstep_controller/GoPosFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_footstep_controller/GoPosFeedback';
[data.PREPLANNING, info.PREPLANNING] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.PLANNING, info.PLANNING] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.WALKING, info.WALKING] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.WAITING, info.WAITING] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.FINISH, info.FINISH] = ros.internal.ros.messages.ros.default_type('uint8',1, 4);
[data.Status, info.Status] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.RemainingSteps, info.RemainingSteps] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'jsk_footstep_controller/GoPosFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'PRE_PLANNING';
info.MatPath{2} = 'PLANNING';
info.MatPath{3} = 'WALKING';
info.MatPath{4} = 'WAITING';
info.MatPath{5} = 'FINISH';
info.MatPath{6} = 'status';
info.MatPath{7} = 'remaining_steps';
