function [data, info] = rawCamData
%RawCamData gives an empty data for multisense_ros/RawCamData

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'multisense_ros/RawCamData';
[data.FramesPerSecond, info.FramesPerSecond] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Gain, info.Gain] = ros.internal.ros.messages.ros.default_type('single',1);
[data.ExposureTime, info.ExposureTime] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.FrameCount, info.FrameCount] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.TimeStamp, info.TimeStamp] = ros.internal.ros.messages.ros.time;
info.TimeStamp.MLdataType = 'struct';
[data.Angle, info.Angle] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Width, info.Width] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.Height, info.Height] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.GrayScaleImage, info.GrayScaleImage] = ros.internal.ros.messages.ros.default_type('uint8',NaN);
[data.DisparityImage, info.DisparityImage] = ros.internal.ros.messages.ros.default_type('uint16',NaN);
info.MessageType = 'multisense_ros/RawCamData';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'frames_per_second';
info.MatPath{2} = 'gain';
info.MatPath{3} = 'exposure_time';
info.MatPath{4} = 'frame_count';
info.MatPath{5} = 'time_stamp';
info.MatPath{6} = 'time_stamp.sec';
info.MatPath{7} = 'time_stamp.nsec';
info.MatPath{8} = 'angle';
info.MatPath{9} = 'width';
info.MatPath{10} = 'height';
info.MatPath{11} = 'gray_scale_image';
info.MatPath{12} = 'disparity_image';
