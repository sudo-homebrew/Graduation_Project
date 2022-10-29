function [data, info] = firmwareInfo
%FirmwareInfo gives an empty data for clearpath_base/FirmwareInfo

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'clearpath_base/FirmwareInfo';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.FirmwareMajor, info.FirmwareMajor] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.FirmwareMinor, info.FirmwareMinor] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.ProtocolMajor, info.ProtocolMajor] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.ProtocolMinor, info.ProtocolMinor] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.FirmwareWriteTime, info.FirmwareWriteTime] = ros.internal.ros.messages.ros.default_type('uint32',1);
info.MessageType = 'clearpath_base/FirmwareInfo';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'firmware_major';
info.MatPath{8} = 'firmware_minor';
info.MatPath{9} = 'protocol_major';
info.MatPath{10} = 'protocol_minor';
info.MatPath{11} = 'firmware_write_time';
