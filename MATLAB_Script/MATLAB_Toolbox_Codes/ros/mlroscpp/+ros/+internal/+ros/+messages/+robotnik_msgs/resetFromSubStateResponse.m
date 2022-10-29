function [data, info] = resetFromSubStateResponse
%ResetFromSubState gives an empty data for robotnik_msgs/ResetFromSubStateResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/ResetFromSubStateResponse';
[data.CurrentSubState, info.CurrentSubState] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Msg, info.Msg] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'robotnik_msgs/ResetFromSubStateResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'currentSubState';
info.MatPath{2} = 'success';
info.MatPath{3} = 'msg';
