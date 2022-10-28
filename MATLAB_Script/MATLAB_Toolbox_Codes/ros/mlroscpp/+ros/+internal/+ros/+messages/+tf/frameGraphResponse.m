function [data, info] = frameGraphResponse
%FrameGraph gives an empty data for tf/FrameGraphResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'tf/FrameGraphResponse';
[data.DotGraph, info.DotGraph] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'tf/FrameGraphResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'dot_graph';
