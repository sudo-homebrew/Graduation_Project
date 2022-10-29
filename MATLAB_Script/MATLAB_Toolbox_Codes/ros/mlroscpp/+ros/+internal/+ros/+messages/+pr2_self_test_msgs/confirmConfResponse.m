function [data, info] = confirmConfResponse
%ConfirmConf gives an empty data for pr2_self_test_msgs/ConfirmConfResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_self_test_msgs/ConfirmConfResponse';
[data.RETRY, info.RETRY] = ros.internal.ros.messages.ros.default_type('int8',1, 0);
[data.FAIL, info.FAIL] = ros.internal.ros.messages.ros.default_type('int8',1, 1);
[data.Retry, info.Retry] = ros.internal.ros.messages.ros.default_type('int8',1);
info.MessageType = 'pr2_self_test_msgs/ConfirmConfResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'RETRY';
info.MatPath{2} = 'FAIL';
info.MatPath{3} = 'retry';
