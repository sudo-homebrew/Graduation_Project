function [data, info] = movingEdgeSettings
%MovingEdgeSettings gives an empty data for visp_tracker/MovingEdgeSettings

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'visp_tracker/MovingEdgeSettings';
[data.MaskSize, info.MaskSize] = ros.internal.ros.messages.ros.default_type('int64',1);
[data.Range, info.Range] = ros.internal.ros.messages.ros.default_type('int64',1);
[data.Threshold, info.Threshold] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Mu1, info.Mu1] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Mu2, info.Mu2] = ros.internal.ros.messages.ros.default_type('double',1);
[data.SampleStep, info.SampleStep] = ros.internal.ros.messages.ros.default_type('int64',1);
[data.Strip, info.Strip] = ros.internal.ros.messages.ros.default_type('int64',1);
[data.FirstThreshold, info.FirstThreshold] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'visp_tracker/MovingEdgeSettings';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'mask_size';
info.MatPath{2} = 'range';
info.MatPath{3} = 'threshold';
info.MatPath{4} = 'mu1';
info.MatPath{5} = 'mu2';
info.MatPath{6} = 'sample_step';
info.MatPath{7} = 'strip';
info.MatPath{8} = 'first_threshold';
