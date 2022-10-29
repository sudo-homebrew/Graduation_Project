function [data, info] = ledAnimRequest
%LedAnim gives an empty data for ardrone_autonomy/LedAnimRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ardrone_autonomy/LedAnimRequest';
[data.Type, info.Type] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Freq, info.Freq] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Duration, info.Duration] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'ardrone_autonomy/LedAnimRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'type';
info.MatPath{2} = 'freq';
info.MatPath{3} = 'duration';
