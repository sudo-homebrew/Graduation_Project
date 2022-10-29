function [data, info] = rawFTDataSample
%RawFTDataSample gives an empty data for ethercat_hardware/RawFTDataSample

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ethercat_hardware/RawFTDataSample';
[data.SampleCount, info.SampleCount] = ros.internal.ros.messages.ros.default_type('uint64',1);
[data.Data, info.Data] = ros.internal.ros.messages.ros.default_type('int16',NaN);
[data.Vhalf, info.Vhalf] = ros.internal.ros.messages.ros.default_type('uint16',1);
info.MessageType = 'ethercat_hardware/RawFTDataSample';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'sample_count';
info.MatPath{2} = 'data';
info.MatPath{3} = 'vhalf';
