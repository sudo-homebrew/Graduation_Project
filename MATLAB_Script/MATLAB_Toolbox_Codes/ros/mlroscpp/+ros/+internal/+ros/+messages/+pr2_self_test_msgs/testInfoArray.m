function [data, info] = testInfoArray
%TestInfoArray gives an empty data for pr2_self_test_msgs/TestInfoArray

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_self_test_msgs/TestInfoArray';
[data.Data, info.Data] = ros.internal.ros.messages.pr2_self_test_msgs.testInfo;
info.Data.MLdataType = 'struct';
info.Data.MaxLen = NaN;
info.Data.MinLen = 0;
data.Data = data.Data([],1);
info.MessageType = 'pr2_self_test_msgs/TestInfoArray';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'data';
info.MatPath{2} = 'data.serial';
info.MatPath{3} = 'data.test_name';
info.MatPath{4} = 'data.test_status';
info.MatPath{5} = 'data.bay_name';
info.MatPath{6} = 'data.machine';
info.MatPath{7} = 'data.board';
info.MatPath{8} = 'data.breaker';
info.MatPath{9} = 'data.power_status';
info.MatPath{10} = 'data.estop';
info.MatPath{11} = 'data.elapsed';
info.MatPath{12} = 'data.status_msg';
