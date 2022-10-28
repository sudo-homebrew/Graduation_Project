function [data, info] = noise
%Noise gives an empty data for stdr_msgs/Noise

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'stdr_msgs/Noise';
[data.Noise_, info.Noise_] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.NoiseMean, info.NoiseMean] = ros.internal.ros.messages.ros.default_type('single',1);
[data.NoiseStd, info.NoiseStd] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'stdr_msgs/Noise';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'noise';
info.MatPath{2} = 'noiseMean';
info.MatPath{3} = 'noiseStd';
