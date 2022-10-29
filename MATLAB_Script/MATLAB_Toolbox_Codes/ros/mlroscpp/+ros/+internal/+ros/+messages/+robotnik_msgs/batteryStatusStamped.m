function [data, info] = batteryStatusStamped
%BatteryStatusStamped gives an empty data for robotnik_msgs/BatteryStatusStamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/BatteryStatusStamped';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Status, info.Status] = ros.internal.ros.messages.robotnik_msgs.batteryStatus;
info.Status.MLdataType = 'struct';
info.MessageType = 'robotnik_msgs/BatteryStatusStamped';
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
info.MatPath{7} = 'status';
info.MatPath{8} = 'status.voltage';
info.MatPath{9} = 'status.current';
info.MatPath{10} = 'status.level';
info.MatPath{11} = 'status.time_remaining';
info.MatPath{12} = 'status.time_charging';
info.MatPath{13} = 'status.is_charging';
