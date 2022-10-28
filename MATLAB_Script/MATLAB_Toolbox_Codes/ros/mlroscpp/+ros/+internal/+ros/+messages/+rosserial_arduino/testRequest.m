function [data, info] = testRequest
%Test gives an empty data for rosserial_arduino/TestRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosserial_arduino/TestRequest';
[data.Input, info.Input] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'rosserial_arduino/TestRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'input';
