function [data, info] = safetyStatus
%SafetyStatus gives an empty data for clearpath_base/SafetyStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'clearpath_base/SafetyStatus';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Flags, info.Flags] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.Estop, info.Estop] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'clearpath_base/SafetyStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'flags';
info.MatPath{8} = 'estop';
