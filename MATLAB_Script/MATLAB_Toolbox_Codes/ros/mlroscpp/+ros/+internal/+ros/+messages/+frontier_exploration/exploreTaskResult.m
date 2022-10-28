function [data, info] = exploreTaskResult
%ExploreTaskResult gives an empty data for frontier_exploration/ExploreTaskResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'frontier_exploration/ExploreTaskResult';
info.MessageType = 'frontier_exploration/ExploreTaskResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
