function [data, info] = frameGraphResponse
%FrameGraph gives an empty data for tf2_msgs/FrameGraphResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'tf2_msgs/FrameGraphResponse';
[data.FrameYaml, info.FrameYaml] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'tf2_msgs/FrameGraphResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'frame_yaml';
