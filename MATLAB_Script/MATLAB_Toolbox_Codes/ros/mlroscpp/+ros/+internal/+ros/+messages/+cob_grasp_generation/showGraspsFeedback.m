function [data, info] = showGraspsFeedback
%ShowGraspsFeedback gives an empty data for cob_grasp_generation/ShowGraspsFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_grasp_generation/ShowGraspsFeedback';
[data.Status, info.Status] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'cob_grasp_generation/ShowGraspsFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'status';
