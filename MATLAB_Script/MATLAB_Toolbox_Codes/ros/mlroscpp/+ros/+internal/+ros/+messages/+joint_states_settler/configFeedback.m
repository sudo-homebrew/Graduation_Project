function [data, info] = configFeedback
%ConfigFeedback gives an empty data for joint_states_settler/ConfigFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'joint_states_settler/ConfigFeedback';
info.MessageType = 'joint_states_settler/ConfigFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
