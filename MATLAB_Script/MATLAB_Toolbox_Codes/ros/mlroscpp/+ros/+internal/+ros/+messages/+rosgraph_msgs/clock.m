function [data, info] = clock
%Clock gives an empty data for rosgraph_msgs/Clock

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosgraph_msgs/Clock';
[data.Clock_, info.Clock_] = ros.internal.ros.messages.ros.time;
info.Clock_.MLdataType = 'struct';
info.MessageType = 'rosgraph_msgs/Clock';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'clock';
info.MatPath{2} = 'clock.sec';
info.MatPath{3} = 'clock.nsec';
