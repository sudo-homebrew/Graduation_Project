function [data, info] = frameGraphRequest
%FrameGraph gives an empty data for tf/FrameGraphRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'tf/FrameGraphRequest';
info.MessageType = 'tf/FrameGraphRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
