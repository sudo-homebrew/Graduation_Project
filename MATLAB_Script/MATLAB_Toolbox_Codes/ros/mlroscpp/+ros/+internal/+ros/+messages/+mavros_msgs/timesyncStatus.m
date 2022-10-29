function [data, info] = timesyncStatus
%TimesyncStatus gives an empty data for mavros_msgs/TimesyncStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/TimesyncStatus';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.RemoteTimestampNs, info.RemoteTimestampNs] = ros.internal.ros.messages.ros.default_type('uint64',1);
[data.ObservedOffsetNs, info.ObservedOffsetNs] = ros.internal.ros.messages.ros.default_type('int64',1);
[data.EstimatedOffsetNs, info.EstimatedOffsetNs] = ros.internal.ros.messages.ros.default_type('int64',1);
[data.RoundTripTimeMs, info.RoundTripTimeMs] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'mavros_msgs/TimesyncStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'remote_timestamp_ns';
info.MatPath{8} = 'observed_offset_ns';
info.MatPath{9} = 'estimated_offset_ns';
info.MatPath{10} = 'round_trip_time_ms';
