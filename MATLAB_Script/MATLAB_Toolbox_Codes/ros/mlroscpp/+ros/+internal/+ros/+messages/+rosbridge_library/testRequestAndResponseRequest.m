function [data, info] = testRequestAndResponseRequest
%TestRequestAndResponse gives an empty data for rosbridge_library/TestRequestAndResponseRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosbridge_library/TestRequestAndResponseRequest';
[data.Data, info.Data] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'rosbridge_library/TestRequestAndResponseRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'data';
