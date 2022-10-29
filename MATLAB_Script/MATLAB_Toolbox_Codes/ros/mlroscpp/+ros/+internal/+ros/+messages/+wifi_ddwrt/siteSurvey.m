function [data, info] = siteSurvey
%SiteSurvey gives an empty data for wifi_ddwrt/SiteSurvey

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'wifi_ddwrt/SiteSurvey';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Networks, info.Networks] = ros.internal.ros.messages.wifi_ddwrt.network;
info.Networks.MLdataType = 'struct';
info.Networks.MaxLen = NaN;
info.Networks.MinLen = 0;
data.Networks = data.Networks([],1);
info.MessageType = 'wifi_ddwrt/SiteSurvey';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,13);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'networks';
info.MatPath{8} = 'networks.macattr';
info.MatPath{9} = 'networks.essid';
info.MatPath{10} = 'networks.channel';
info.MatPath{11} = 'networks.rssi';
info.MatPath{12} = 'networks.noise';
info.MatPath{13} = 'networks.beacon';
