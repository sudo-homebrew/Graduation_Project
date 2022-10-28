function [data, info] = advertiseRequest
%Advertise gives an empty data for gateway_msgs/AdvertiseRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gateway_msgs/AdvertiseRequest';
[data.Cancel, info.Cancel] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Rules, info.Rules] = ros.internal.ros.messages.gateway_msgs.rule;
info.Rules.MLdataType = 'struct';
info.Rules.MaxLen = NaN;
info.Rules.MinLen = 0;
data.Rules = data.Rules([],1);
info.MessageType = 'gateway_msgs/AdvertiseRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'cancel';
info.MatPath{2} = 'rules';
info.MatPath{3} = 'rules.type';
info.MatPath{4} = 'rules.name';
info.MatPath{5} = 'rules.node';
