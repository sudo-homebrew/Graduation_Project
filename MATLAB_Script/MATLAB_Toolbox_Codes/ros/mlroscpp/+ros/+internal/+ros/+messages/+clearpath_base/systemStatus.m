function [data, info] = systemStatus
%SystemStatus gives an empty data for clearpath_base/SystemStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'clearpath_base/SystemStatus';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Uptime, info.Uptime] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Voltages, info.Voltages] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.Currents, info.Currents] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.Temperatures, info.Temperatures] = ros.internal.ros.messages.ros.default_type('double',NaN);
info.MessageType = 'clearpath_base/SystemStatus';
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
info.MatPath{7} = 'uptime';
info.MatPath{8} = 'voltages';
info.MatPath{9} = 'currents';
info.MatPath{10} = 'temperatures';
