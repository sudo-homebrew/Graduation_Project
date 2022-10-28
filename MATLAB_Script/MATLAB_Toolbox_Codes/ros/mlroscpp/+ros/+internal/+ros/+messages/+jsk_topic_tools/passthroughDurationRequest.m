function [data, info] = passthroughDurationRequest
%PassthroughDuration gives an empty data for jsk_topic_tools/PassthroughDurationRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_topic_tools/PassthroughDurationRequest';
[data.Duration, info.Duration] = ros.internal.ros.messages.ros.duration;
info.Duration.MLdataType = 'struct';
info.MessageType = 'jsk_topic_tools/PassthroughDurationRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'duration';
info.MatPath{2} = 'duration.sec';
info.MatPath{3} = 'duration.nsec';
