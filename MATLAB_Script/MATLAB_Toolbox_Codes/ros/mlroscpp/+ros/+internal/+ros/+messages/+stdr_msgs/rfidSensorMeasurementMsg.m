function [data, info] = rfidSensorMeasurementMsg
%RfidSensorMeasurementMsg gives an empty data for stdr_msgs/RfidSensorMeasurementMsg

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'stdr_msgs/RfidSensorMeasurementMsg';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.RfidTagsIds, info.RfidTagsIds] = ros.internal.ros.messages.ros.char('string',NaN);
[data.RfidTagsMsgs, info.RfidTagsMsgs] = ros.internal.ros.messages.ros.char('string',NaN);
[data.RfidTagsDbs, info.RfidTagsDbs] = ros.internal.ros.messages.ros.default_type('single',NaN);
info.MessageType = 'stdr_msgs/RfidSensorMeasurementMsg';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'rfid_tags_ids';
info.MatPath{8} = 'rfid_tags_msgs';
info.MatPath{9} = 'rfid_tags_dbs';
