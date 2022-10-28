function [data, info] = intervalStamped
%IntervalStamped gives an empty data for calibration_msgs/IntervalStamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'calibration_msgs/IntervalStamped';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Interval, info.Interval] = ros.internal.ros.messages.calibration_msgs.interval;
info.Interval.MLdataType = 'struct';
info.MessageType = 'calibration_msgs/IntervalStamped';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,13);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'interval';
info.MatPath{8} = 'interval.start';
info.MatPath{9} = 'interval.start.sec';
info.MatPath{10} = 'interval.start.nsec';
info.MatPath{11} = 'interval.end';
info.MatPath{12} = 'interval.end.sec';
info.MatPath{13} = 'interval.end.nsec';
