function [data, info] = connectionType
%ConnectionType gives an empty data for gateway_msgs/ConnectionType

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gateway_msgs/ConnectionType';
[data.PUBLISHER, info.PUBLISHER] = ros.internal.ros.messages.ros.char('string',0);
[data.PUBLISHER, info.PUBLISHER] = ros.internal.ros.messages.ros.char('string',1,'publisher');
[data.SUBSCRIBER, info.SUBSCRIBER] = ros.internal.ros.messages.ros.char('string',0);
[data.SUBSCRIBER, info.SUBSCRIBER] = ros.internal.ros.messages.ros.char('string',1,'subscriber');
[data.SERVICE, info.SERVICE] = ros.internal.ros.messages.ros.char('string',0);
[data.SERVICE, info.SERVICE] = ros.internal.ros.messages.ros.char('string',1,'service');
[data.ACTIONCLIENT, info.ACTIONCLIENT] = ros.internal.ros.messages.ros.char('string',0);
[data.ACTIONCLIENT, info.ACTIONCLIENT] = ros.internal.ros.messages.ros.char('string',1,'action_client');
[data.ACTIONSERVER, info.ACTIONSERVER] = ros.internal.ros.messages.ros.char('string',0);
[data.ACTIONSERVER, info.ACTIONSERVER] = ros.internal.ros.messages.ros.char('string',1,'action_server');
[data.INVALID, info.INVALID] = ros.internal.ros.messages.ros.char('string',0);
[data.INVALID, info.INVALID] = ros.internal.ros.messages.ros.char('string',1,'invalid');
info.MessageType = 'gateway_msgs/ConnectionType';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'PUBLISHER';
info.MatPath{2} = 'SUBSCRIBER';
info.MatPath{3} = 'SERVICE';
info.MatPath{4} = 'ACTION_CLIENT';
info.MatPath{5} = 'ACTION_SERVER';
info.MatPath{6} = 'INVALID';
