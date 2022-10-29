function [data, info] = interval
%Interval gives an empty data for calibration_msgs/Interval

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'calibration_msgs/Interval';
[data.Start, info.Start] = ros.internal.ros.messages.ros.time;
info.Start.MLdataType = 'struct';
[data.End, info.End] = ros.internal.ros.messages.ros.time;
info.End.MLdataType = 'struct';
info.MessageType = 'calibration_msgs/Interval';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'start';
info.MatPath{2} = 'start.sec';
info.MatPath{3} = 'start.nsec';
info.MatPath{4} = 'end';
info.MatPath{5} = 'end.sec';
info.MatPath{6} = 'end.nsec';
