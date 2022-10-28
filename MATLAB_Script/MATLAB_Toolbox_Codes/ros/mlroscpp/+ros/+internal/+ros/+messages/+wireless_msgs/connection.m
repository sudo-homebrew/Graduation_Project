function [data, info] = connection
%Connection gives an empty data for wireless_msgs/Connection

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'wireless_msgs/Connection';
[data.Bitrate, info.Bitrate] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Txpower, info.Txpower] = ros.internal.ros.messages.ros.default_type('int16',1);
[data.LinkQualityRaw, info.LinkQualityRaw] = ros.internal.ros.messages.ros.char('string',0);
[data.LinkQuality, info.LinkQuality] = ros.internal.ros.messages.ros.default_type('single',1);
[data.SignalLevel, info.SignalLevel] = ros.internal.ros.messages.ros.default_type('int16',1);
[data.NoiseLevel, info.NoiseLevel] = ros.internal.ros.messages.ros.default_type('int16',1);
[data.Essid, info.Essid] = ros.internal.ros.messages.ros.char('string',0);
[data.Bssid, info.Bssid] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'wireless_msgs/Connection';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'bitrate';
info.MatPath{2} = 'txpower';
info.MatPath{3} = 'link_quality_raw';
info.MatPath{4} = 'link_quality';
info.MatPath{5} = 'signal_level';
info.MatPath{6} = 'noise_level';
info.MatPath{7} = 'essid';
info.MatPath{8} = 'bssid';
