function [data, info] = batteryDockingStatusStamped
%BatteryDockingStatusStamped gives an empty data for robotnik_msgs/BatteryDockingStatusStamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/BatteryDockingStatusStamped';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Status, info.Status] = ros.internal.ros.messages.robotnik_msgs.batteryDockingStatus;
info.Status.MLdataType = 'struct';
info.MessageType = 'robotnik_msgs/BatteryDockingStatusStamped';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'status';
info.MatPath{8} = 'status.MODE_DISABLED';
info.MatPath{9} = 'status.MODE_AUTO_HW';
info.MatPath{10} = 'status.MODE_AUTO_SW';
info.MatPath{11} = 'status.MODE_MANUAL_SW';
info.MatPath{12} = 'status.operation_mode';
info.MatPath{13} = 'status.contact_relay_status';
info.MatPath{14} = 'status.charger_relay_status';
