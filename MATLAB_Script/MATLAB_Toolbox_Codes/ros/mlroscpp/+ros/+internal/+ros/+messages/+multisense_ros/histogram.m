function [data, info] = histogram
%Histogram gives an empty data for multisense_ros/Histogram

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'multisense_ros/Histogram';
[data.FrameCount, info.FrameCount] = ros.internal.ros.messages.ros.default_type('int64',1);
[data.TimeStamp, info.TimeStamp] = ros.internal.ros.messages.ros.time;
info.TimeStamp.MLdataType = 'struct';
[data.Width, info.Width] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Height, info.Height] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Gain, info.Gain] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Fps, info.Fps] = ros.internal.ros.messages.ros.default_type('single',1);
[data.ExposureTime, info.ExposureTime] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Channels, info.Channels] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Bins, info.Bins] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Data, info.Data] = ros.internal.ros.messages.ros.default_type('uint32',NaN);
info.MessageType = 'multisense_ros/Histogram';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'frame_count';
info.MatPath{2} = 'time_stamp';
info.MatPath{3} = 'time_stamp.sec';
info.MatPath{4} = 'time_stamp.nsec';
info.MatPath{5} = 'width';
info.MatPath{6} = 'height';
info.MatPath{7} = 'gain';
info.MatPath{8} = 'fps';
info.MatPath{9} = 'exposure_time';
info.MatPath{10} = 'channels';
info.MatPath{11} = 'bins';
info.MatPath{12} = 'data';
