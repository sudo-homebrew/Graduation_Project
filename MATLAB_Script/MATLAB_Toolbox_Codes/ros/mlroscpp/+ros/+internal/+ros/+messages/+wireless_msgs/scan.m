function [data, info] = scan
%Scan gives an empty data for wireless_msgs/Scan

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'wireless_msgs/Scan';
[data.Networks, info.Networks] = ros.internal.ros.messages.wireless_msgs.network;
info.Networks.MLdataType = 'struct';
info.Networks.MaxLen = NaN;
info.Networks.MinLen = 0;
data.Networks = data.Networks([],1);
info.MessageType = 'wireless_msgs/Scan';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'networks';
info.MatPath{2} = 'networks.type';
info.MatPath{3} = 'networks.essid';
info.MatPath{4} = 'networks.mac';
info.MatPath{5} = 'networks.mode';
info.MatPath{6} = 'networks.frequency';
info.MatPath{7} = 'networks.encryption';
