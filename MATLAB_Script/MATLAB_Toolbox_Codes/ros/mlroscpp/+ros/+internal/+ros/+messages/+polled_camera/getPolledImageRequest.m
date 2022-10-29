function [data, info] = getPolledImageRequest
%GetPolledImage gives an empty data for polled_camera/GetPolledImageRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'polled_camera/GetPolledImageRequest';
[data.ResponseNamespace, info.ResponseNamespace] = ros.internal.ros.messages.ros.char('string',0);
[data.Timeout, info.Timeout] = ros.internal.ros.messages.ros.duration;
info.Timeout.MLdataType = 'struct';
[data.BinningX, info.BinningX] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.BinningY, info.BinningY] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Roi, info.Roi] = ros.internal.ros.messages.sensor_msgs.regionOfInterest;
info.Roi.MLdataType = 'struct';
info.MessageType = 'polled_camera/GetPolledImageRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'response_namespace';
info.MatPath{2} = 'timeout';
info.MatPath{3} = 'timeout.sec';
info.MatPath{4} = 'timeout.nsec';
info.MatPath{5} = 'binning_x';
info.MatPath{6} = 'binning_y';
info.MatPath{7} = 'roi';
info.MatPath{8} = 'roi.x_offset';
info.MatPath{9} = 'roi.y_offset';
info.MatPath{10} = 'roi.height';
info.MatPath{11} = 'roi.width';
info.MatPath{12} = 'roi.do_rectify';
