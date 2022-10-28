function [data, info] = testInfo
%TestInfo gives an empty data for pr2_self_test_msgs/TestInfo

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_self_test_msgs/TestInfo';
[data.Serial, info.Serial] = ros.internal.ros.messages.ros.char('string',0);
[data.TestName, info.TestName] = ros.internal.ros.messages.ros.char('string',0);
[data.TestStatus, info.TestStatus] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.BayName, info.BayName] = ros.internal.ros.messages.ros.char('string',0);
[data.Machine, info.Machine] = ros.internal.ros.messages.ros.char('string',0);
[data.Board, info.Board] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Breaker, info.Breaker] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.PowerStatus, info.PowerStatus] = ros.internal.ros.messages.ros.char('string',0);
[data.Estop, info.Estop] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.Elapsed, info.Elapsed] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.StatusMsg, info.StatusMsg] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'pr2_self_test_msgs/TestInfo';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'serial';
info.MatPath{2} = 'test_name';
info.MatPath{3} = 'test_status';
info.MatPath{4} = 'bay_name';
info.MatPath{5} = 'machine';
info.MatPath{6} = 'board';
info.MatPath{7} = 'breaker';
info.MatPath{8} = 'power_status';
info.MatPath{9} = 'estop';
info.MatPath{10} = 'elapsed';
info.MatPath{11} = 'status_msg';
