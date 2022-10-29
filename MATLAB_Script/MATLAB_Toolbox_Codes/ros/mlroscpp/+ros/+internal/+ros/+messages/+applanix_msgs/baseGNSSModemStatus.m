function [data, info] = baseGNSSModemStatus
%BaseGNSSModemStatus gives an empty data for applanix_msgs/BaseGNSSModemStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/BaseGNSSModemStatus';
[data.Td, info.Td] = ros.internal.ros.messages.applanix_msgs.timeDistance;
info.Td.MLdataType = 'struct';
[data.ModemResponse, info.ModemResponse] = ros.internal.ros.messages.ros.default_type('uint8',16);
[data.ConnectionStatus, info.ConnectionStatus] = ros.internal.ros.messages.ros.default_type('uint8',48);
[data.RedialsPerDisconnect, info.RedialsPerDisconnect] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.MaxRedialsPerDisconnect, info.MaxRedialsPerDisconnect] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.NumDisconnects, info.NumDisconnects] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.DataGapLength, info.DataGapLength] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.MaxDataGapLength, info.MaxDataGapLength] = ros.internal.ros.messages.ros.default_type('uint32',1);
info.MessageType = 'applanix_msgs/BaseGNSSModemStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,13);
info.MatPath{1} = 'td';
info.MatPath{2} = 'td.time1';
info.MatPath{3} = 'td.time2';
info.MatPath{4} = 'td.distance';
info.MatPath{5} = 'td.time_types';
info.MatPath{6} = 'td.distance_type';
info.MatPath{7} = 'modem_response';
info.MatPath{8} = 'connection_status';
info.MatPath{9} = 'redials_per_disconnect';
info.MatPath{10} = 'max_redials_per_disconnect';
info.MatPath{11} = 'num_disconnects';
info.MatPath{12} = 'data_gap_length';
info.MatPath{13} = 'max_data_gap_length';
