function [data, info] = exposureSequence
%ExposureSequence gives an empty data for image_exposure_msgs/ExposureSequence

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'image_exposure_msgs/ExposureSequence';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Shutter, info.Shutter] = ros.internal.ros.messages.ros.default_type('uint32',NaN);
[data.Gain, info.Gain] = ros.internal.ros.messages.ros.default_type('single',1);
[data.WhiteBalanceBlue, info.WhiteBalanceBlue] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.WhiteBalanceRed, info.WhiteBalanceRed] = ros.internal.ros.messages.ros.default_type('uint16',1);
info.MessageType = 'image_exposure_msgs/ExposureSequence';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'shutter';
info.MatPath{8} = 'gain';
info.MatPath{9} = 'white_balance_blue';
info.MatPath{10} = 'white_balance_red';