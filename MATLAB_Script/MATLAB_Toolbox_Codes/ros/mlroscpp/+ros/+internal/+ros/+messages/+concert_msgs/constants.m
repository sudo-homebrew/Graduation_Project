function [data, info] = constants
%Constants gives an empty data for concert_msgs/Constants

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'concert_msgs/Constants';
[data.CONCERTCLIENTSTATUSAVAILABLE, info.CONCERTCLIENTSTATUSAVAILABLE] = ros.internal.ros.messages.ros.char('string',0);
[data.CONCERTCLIENTSTATUSAVAILABLE, info.CONCERTCLIENTSTATUSAVAILABLE] = ros.internal.ros.messages.ros.char('string',1,'available');
[data.CONCERTCLIENTSTATUSUNVAILABLE, info.CONCERTCLIENTSTATUSUNVAILABLE] = ros.internal.ros.messages.ros.char('string',0);
[data.CONCERTCLIENTSTATUSUNVAILABLE, info.CONCERTCLIENTSTATUSUNVAILABLE] = ros.internal.ros.messages.ros.char('string',1,'unavailable');
[data.CONCERTCLIENTSTATUSCONNECTED, info.CONCERTCLIENTSTATUSCONNECTED] = ros.internal.ros.messages.ros.char('string',0);
[data.CONCERTCLIENTSTATUSCONNECTED, info.CONCERTCLIENTSTATUSCONNECTED] = ros.internal.ros.messages.ros.char('string',1,'connected');
[data.APPSTATUSSTOPPED, info.APPSTATUSSTOPPED] = ros.internal.ros.messages.ros.char('string',0);
[data.APPSTATUSSTOPPED, info.APPSTATUSSTOPPED] = ros.internal.ros.messages.ros.char('string',1,'stopped');
[data.APPSTATUSRUNNING, info.APPSTATUSRUNNING] = ros.internal.ros.messages.ros.char('string',0);
[data.APPSTATUSRUNNING, info.APPSTATUSRUNNING] = ros.internal.ros.messages.ros.char('string',1,'running');
info.MessageType = 'concert_msgs/Constants';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'CONCERT_CLIENT_STATUS_AVAILABLE';
info.MatPath{2} = 'CONCERT_CLIENT_STATUS_UNVAILABLE';
info.MatPath{3} = 'CONCERT_CLIENT_STATUS_CONNECTED';
info.MatPath{4} = 'APP_STATUS_STOPPED';
info.MatPath{5} = 'APP_STATUS_RUNNING';
