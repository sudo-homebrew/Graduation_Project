function [data, info] = remoteGatewayInfoRequest
%RemoteGatewayInfo gives an empty data for gateway_msgs/RemoteGatewayInfoRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gateway_msgs/RemoteGatewayInfoRequest';
[data.Gateways, info.Gateways] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'gateway_msgs/RemoteGatewayInfoRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'gateways';
