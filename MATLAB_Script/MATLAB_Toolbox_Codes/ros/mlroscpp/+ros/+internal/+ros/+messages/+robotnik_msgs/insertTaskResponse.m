function [data, info] = insertTaskResponse
%InsertTask gives an empty data for robotnik_msgs/InsertTaskResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/InsertTaskResponse';
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Msg, info.Msg] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'robotnik_msgs/InsertTaskResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'success';
info.MatPath{2} = 'msg';
