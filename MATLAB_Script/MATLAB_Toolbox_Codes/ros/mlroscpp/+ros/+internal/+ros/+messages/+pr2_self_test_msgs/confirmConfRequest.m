function [data, info] = confirmConfRequest
%ConfirmConf gives an empty data for pr2_self_test_msgs/ConfirmConfRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_self_test_msgs/ConfirmConfRequest';
[data.Message, info.Message] = ros.internal.ros.messages.ros.char('string',0);
[data.Details, info.Details] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'pr2_self_test_msgs/ConfirmConfRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'message';
info.MatPath{2} = 'details';
