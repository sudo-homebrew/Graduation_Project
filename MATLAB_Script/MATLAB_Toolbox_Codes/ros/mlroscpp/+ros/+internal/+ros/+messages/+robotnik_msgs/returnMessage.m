function [data, info] = returnMessage
%ReturnMessage gives an empty data for robotnik_msgs/ReturnMessage

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/ReturnMessage';
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Message, info.Message] = ros.internal.ros.messages.ros.char('string',0);
[data.Code, info.Code] = ros.internal.ros.messages.ros.default_type('int16',1);
info.MessageType = 'robotnik_msgs/ReturnMessage';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'success';
info.MatPath{2} = 'message';
info.MatPath{3} = 'code';
