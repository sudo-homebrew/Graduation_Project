function [data, info] = requestParamResponse
%RequestParam gives an empty data for rosserial_msgs/RequestParamResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosserial_msgs/RequestParamResponse';
[data.Ints, info.Ints] = ros.internal.ros.messages.ros.default_type('int32',NaN);
[data.Floats, info.Floats] = ros.internal.ros.messages.ros.default_type('single',NaN);
[data.Strings, info.Strings] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'rosserial_msgs/RequestParamResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'ints';
info.MatPath{2} = 'floats';
info.MatPath{3} = 'strings';
