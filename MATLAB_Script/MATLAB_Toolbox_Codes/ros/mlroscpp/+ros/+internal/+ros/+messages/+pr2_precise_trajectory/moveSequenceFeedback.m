function [data, info] = moveSequenceFeedback
%MoveSequenceFeedback gives an empty data for pr2_precise_trajectory/MoveSequenceFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_precise_trajectory/MoveSequenceFeedback';
[data.PoseIndex, info.PoseIndex] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.PercentComplete, info.PercentComplete] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'pr2_precise_trajectory/MoveSequenceFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'pose_index';
info.MatPath{2} = 'percent_complete';
