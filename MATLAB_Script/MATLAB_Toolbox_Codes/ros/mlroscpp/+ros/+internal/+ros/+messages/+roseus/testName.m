function [data, info] = testName
%TestName gives an empty data for roseus/TestName

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'roseus/TestName';
[data.Name, info.Name] = ros.internal.ros.messages.roseus.stringStamped;
info.Name.MLdataType = 'struct';
info.MessageType = 'roseus/TestName';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'name';
info.MatPath{2} = 'name.header';
info.MatPath{3} = 'name.header.seq';
info.MatPath{4} = 'name.header.stamp';
info.MatPath{5} = 'name.header.stamp.sec';
info.MatPath{6} = 'name.header.stamp.nsec';
info.MatPath{7} = 'name.header.frame_id';
info.MatPath{8} = 'name.data';
