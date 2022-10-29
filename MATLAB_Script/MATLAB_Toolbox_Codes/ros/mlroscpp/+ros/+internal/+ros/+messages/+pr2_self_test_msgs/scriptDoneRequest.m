function [data, info] = scriptDoneRequest
%ScriptDone gives an empty data for pr2_self_test_msgs/ScriptDoneRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_self_test_msgs/ScriptDoneRequest';
[data.RESULTOK, info.RESULTOK] = ros.internal.ros.messages.ros.default_type('int8',1, 0);
[data.RESULTFAIL, info.RESULTFAIL] = ros.internal.ros.messages.ros.default_type('int8',1, 1);
[data.RESULTERROR, info.RESULTERROR] = ros.internal.ros.messages.ros.default_type('int8',1, 2);
[data.Result, info.Result] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.FailureMsg, info.FailureMsg] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'pr2_self_test_msgs/ScriptDoneRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'RESULT_OK';
info.MatPath{2} = 'RESULT_FAIL';
info.MatPath{3} = 'RESULT_ERROR';
info.MatPath{4} = 'result';
info.MatPath{5} = 'failure_msg';
