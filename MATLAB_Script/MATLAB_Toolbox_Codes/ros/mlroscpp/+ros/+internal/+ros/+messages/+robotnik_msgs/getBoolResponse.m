function [data, info] = getBoolResponse
%GetBool gives an empty data for robotnik_msgs/GetBoolResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/GetBoolResponse';
[data.Data, info.Data] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Message, info.Message] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'robotnik_msgs/GetBoolResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'data';
info.MatPath{2} = 'success';
info.MatPath{3} = 'message';
