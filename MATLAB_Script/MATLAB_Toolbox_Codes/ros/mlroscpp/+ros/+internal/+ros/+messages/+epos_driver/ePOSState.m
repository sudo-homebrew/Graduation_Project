function [data, info] = ePOSState
%EPOSState gives an empty data for epos_driver/EPOSState

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'epos_driver/EPOSState';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.RawPosition, info.RawPosition] = ros.internal.ros.messages.ros.default_type('int64',1);
[data.Position, info.Position] = ros.internal.ros.messages.ros.default_type('double',1);
[data.RawSpeed, info.RawSpeed] = ros.internal.ros.messages.ros.default_type('int64',1);
[data.Speed, info.Speed] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Acceleration, info.Acceleration] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Current, info.Current] = ros.internal.ros.messages.ros.default_type('int16',1);
[data.Sync, info.Sync] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'epos_driver/EPOSState';
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
info.MatPath{7} = 'raw_position';
info.MatPath{8} = 'position';
info.MatPath{9} = 'raw_speed';
info.MatPath{10} = 'speed';
info.MatPath{11} = 'acceleration';
info.MatPath{12} = 'current';
info.MatPath{13} = 'sync';
