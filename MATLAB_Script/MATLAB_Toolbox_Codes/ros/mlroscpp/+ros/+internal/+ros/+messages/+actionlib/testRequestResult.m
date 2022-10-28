function [data, info] = testRequestResult
%TestRequestResult gives an empty data for actionlib/TestRequestResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'actionlib/TestRequestResult';
[data.TheResult, info.TheResult] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.IsSimpleServer, info.IsSimpleServer] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'actionlib/TestRequestResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'the_result';
info.MatPath{2} = 'is_simple_server';
