function [data, info] = rawFTData
%RawFTData gives an empty data for ethercat_hardware/RawFTData

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ethercat_hardware/RawFTData';
[data.Samples, info.Samples] = ros.internal.ros.messages.ethercat_hardware.rawFTDataSample;
info.Samples.MLdataType = 'struct';
info.Samples.MaxLen = NaN;
info.Samples.MinLen = 0;
data.Samples = data.Samples([],1);
[data.SampleCount, info.SampleCount] = ros.internal.ros.messages.ros.default_type('int64',1);
[data.MissedSamples, info.MissedSamples] = ros.internal.ros.messages.ros.default_type('int64',1);
info.MessageType = 'ethercat_hardware/RawFTData';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'samples';
info.MatPath{2} = 'samples.sample_count';
info.MatPath{3} = 'samples.data';
info.MatPath{4} = 'samples.vhalf';
info.MatPath{5} = 'sample_count';
info.MatPath{6} = 'missed_samples';
