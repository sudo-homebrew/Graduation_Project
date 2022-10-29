function [data, info] = powerStatus
%PowerStatus gives an empty data for clearpath_base/PowerStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'clearpath_base/PowerStatus';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Sources, info.Sources] = ros.internal.ros.messages.clearpath_base.powerSource;
info.Sources.MLdataType = 'struct';
info.Sources.MaxLen = NaN;
info.Sources.MinLen = 0;
data.Sources = data.Sources([],1);
info.MessageType = 'clearpath_base/PowerStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'sources';
info.MatPath{8} = 'sources.charge';
info.MatPath{9} = 'sources.capacity';
info.MatPath{10} = 'sources.present';
info.MatPath{11} = 'sources.in_use';
info.MatPath{12} = 'sources.description';
