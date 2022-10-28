function [data, info] = gNSSDGPSChannelStatus
%GNSSDGPSChannelStatus gives an empty data for applanix_msgs/GNSSDGPSChannelStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/GNSSDGPSChannelStatus';
[data.Frequency, info.Frequency] = ros.internal.ros.messages.ros.default_type('double',1);
[data.AcquisitionMode, info.AcquisitionMode] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Status, info.Status] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Rtcm, info.Rtcm] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Snr, info.Snr] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.DataRate, info.DataRate] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Lock, info.Lock] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.DgpsSourceAutoSwitching, info.DgpsSourceAutoSwitching] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Provider, info.Provider] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'applanix_msgs/GNSSDGPSChannelStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'frequency';
info.MatPath{2} = 'acquisition_mode';
info.MatPath{3} = 'status';
info.MatPath{4} = 'rtcm';
info.MatPath{5} = 'snr';
info.MatPath{6} = 'data_rate';
info.MatPath{7} = 'lock';
info.MatPath{8} = 'dgps_source_auto_switching';
info.MatPath{9} = 'provider';
