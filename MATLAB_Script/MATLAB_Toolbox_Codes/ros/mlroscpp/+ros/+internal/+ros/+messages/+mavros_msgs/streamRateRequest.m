function [data, info] = streamRateRequest
%StreamRate gives an empty data for mavros_msgs/StreamRateRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/StreamRateRequest';
[data.STREAMALL, info.STREAMALL] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.STREAMRAWSENSORS, info.STREAMRAWSENSORS] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.STREAMEXTENDEDSTATUS, info.STREAMEXTENDEDSTATUS] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.STREAMRCCHANNELS, info.STREAMRCCHANNELS] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.STREAMRAWCONTROLLER, info.STREAMRAWCONTROLLER] = ros.internal.ros.messages.ros.default_type('uint8',1, 4);
[data.STREAMPOSITION, info.STREAMPOSITION] = ros.internal.ros.messages.ros.default_type('uint8',1, 6);
[data.STREAMEXTRA1, info.STREAMEXTRA1] = ros.internal.ros.messages.ros.default_type('uint8',1, 10);
[data.STREAMEXTRA2, info.STREAMEXTRA2] = ros.internal.ros.messages.ros.default_type('uint8',1, 11);
[data.STREAMEXTRA3, info.STREAMEXTRA3] = ros.internal.ros.messages.ros.default_type('uint8',1, 12);
[data.StreamId, info.StreamId] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.MessageRate, info.MessageRate] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.OnOff, info.OnOff] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'mavros_msgs/StreamRateRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'STREAM_ALL';
info.MatPath{2} = 'STREAM_RAW_SENSORS';
info.MatPath{3} = 'STREAM_EXTENDED_STATUS';
info.MatPath{4} = 'STREAM_RC_CHANNELS';
info.MatPath{5} = 'STREAM_RAW_CONTROLLER';
info.MatPath{6} = 'STREAM_POSITION';
info.MatPath{7} = 'STREAM_EXTRA1';
info.MatPath{8} = 'STREAM_EXTRA2';
info.MatPath{9} = 'STREAM_EXTRA3';
info.MatPath{10} = 'stream_id';
info.MatPath{11} = 'message_rate';
info.MatPath{12} = 'on_off';
