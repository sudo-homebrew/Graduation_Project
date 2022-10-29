function [data, info] = dummy
%Dummy gives an empty data for bride_tutorials/Dummy

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'bride_tutorials/Dummy';
[data.Test, info.Test] = ros.internal.ros.messages.std_msgs.string;
info.Test.MLdataType = 'struct';
info.MessageType = 'bride_tutorials/Dummy';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'test';
info.MatPath{2} = 'test.data';
