function [data, info] = verifyObjectResponse
%VerifyObject gives an empty data for hector_worldmodel_msgs/VerifyObjectResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'hector_worldmodel_msgs/VerifyObjectResponse';
[data.Response, info.Response] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.UNKNOWN, info.UNKNOWN] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.DISCARD, info.DISCARD] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.CONFIRM, info.CONFIRM] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
info.MessageType = 'hector_worldmodel_msgs/VerifyObjectResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'response';
info.MatPath{2} = 'UNKNOWN';
info.MatPath{3} = 'DISCARD';
info.MatPath{4} = 'CONFIRM';
