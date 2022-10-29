function [data, info] = time
%Time gives an empty data for std_msgs/Time

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'std_msgs/Time';
[data.Data, info.Data] = ros.internal.ros.messages.ros.time;
info.Data.MLdataType = 'struct';
info.MessageType = 'std_msgs/Time';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'data';
info.MatPath{2} = 'data.sec';
info.MatPath{3} = 'data.nsec';
