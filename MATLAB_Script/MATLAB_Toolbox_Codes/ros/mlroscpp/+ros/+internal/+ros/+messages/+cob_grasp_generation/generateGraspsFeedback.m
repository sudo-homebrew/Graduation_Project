function [data, info] = generateGraspsFeedback
%GenerateGraspsFeedback gives an empty data for cob_grasp_generation/GenerateGraspsFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_grasp_generation/GenerateGraspsFeedback';
[data.Status, info.Status] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'cob_grasp_generation/GenerateGraspsFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'status';
