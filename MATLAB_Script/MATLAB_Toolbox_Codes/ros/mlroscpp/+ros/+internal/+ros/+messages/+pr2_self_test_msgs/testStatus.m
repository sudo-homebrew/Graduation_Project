function [data, info] = testStatus
%TestStatus gives an empty data for pr2_self_test_msgs/TestStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_self_test_msgs/TestStatus';
[data.TESTRUNNING, info.TESTRUNNING] = ros.internal.ros.messages.ros.default_type('int8',1, 0);
[data.TESTWARNING, info.TESTWARNING] = ros.internal.ros.messages.ros.default_type('int8',1, 1);
[data.TESTERROR, info.TESTERROR] = ros.internal.ros.messages.ros.default_type('int8',1, 2);
[data.TESTSTALE, info.TESTSTALE] = ros.internal.ros.messages.ros.default_type('int8',1, 3);
[data.RUNNING, info.RUNNING] = ros.internal.ros.messages.ros.default_type('int8',1, 0);
[data.WARNING, info.WARNING] = ros.internal.ros.messages.ros.default_type('int8',1, 1);
[data.ERROR, info.ERROR] = ros.internal.ros.messages.ros.default_type('int8',1, 2);
[data.STALE, info.STALE] = ros.internal.ros.messages.ros.default_type('int8',1, 3);
[data.TestOk, info.TestOk] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.Message, info.Message] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'pr2_self_test_msgs/TestStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'TEST_RUNNING';
info.MatPath{2} = 'TEST_WARNING';
info.MatPath{3} = 'TEST_ERROR';
info.MatPath{4} = 'TEST_STALE';
info.MatPath{5} = 'RUNNING';
info.MatPath{6} = 'WARNING';
info.MatPath{7} = 'ERROR';
info.MatPath{8} = 'STALE';
info.MatPath{9} = 'test_ok';
info.MatPath{10} = 'message';
