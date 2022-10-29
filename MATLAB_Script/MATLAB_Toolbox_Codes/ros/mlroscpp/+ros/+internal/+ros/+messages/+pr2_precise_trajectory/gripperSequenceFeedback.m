function [data, info] = gripperSequenceFeedback
%GripperSequenceFeedback gives an empty data for pr2_precise_trajectory/GripperSequenceFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_precise_trajectory/GripperSequenceFeedback';
info.MessageType = 'pr2_precise_trajectory/GripperSequenceFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
