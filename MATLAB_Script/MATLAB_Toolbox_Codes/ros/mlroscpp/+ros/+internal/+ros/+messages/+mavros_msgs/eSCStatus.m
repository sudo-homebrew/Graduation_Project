function [data, info] = eSCStatus
%ESCStatus gives an empty data for mavros_msgs/ESCStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/ESCStatus';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.EscStatus, info.EscStatus] = ros.internal.ros.messages.mavros_msgs.eSCStatusItem;
info.EscStatus.MLdataType = 'struct';
info.EscStatus.MaxLen = NaN;
info.EscStatus.MinLen = 0;
data.EscStatus = data.EscStatus([],1);
info.MessageType = 'mavros_msgs/ESCStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,16);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'esc_status';
info.MatPath{8} = 'esc_status.header';
info.MatPath{9} = 'esc_status.header.seq';
info.MatPath{10} = 'esc_status.header.stamp';
info.MatPath{11} = 'esc_status.header.stamp.sec';
info.MatPath{12} = 'esc_status.header.stamp.nsec';
info.MatPath{13} = 'esc_status.header.frame_id';
info.MatPath{14} = 'esc_status.rpm';
info.MatPath{15} = 'esc_status.voltage';
info.MatPath{16} = 'esc_status.current';
