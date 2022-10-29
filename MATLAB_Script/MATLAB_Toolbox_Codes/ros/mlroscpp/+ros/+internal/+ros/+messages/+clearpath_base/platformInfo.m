function [data, info] = platformInfo
%PlatformInfo gives an empty data for clearpath_base/PlatformInfo

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'clearpath_base/PlatformInfo';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Model, info.Model] = ros.internal.ros.messages.ros.char('string',0);
[data.Revision, info.Revision] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.Serial, info.Serial] = ros.internal.ros.messages.ros.default_type('uint32',1);
info.MessageType = 'clearpath_base/PlatformInfo';
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
info.MatPath{7} = 'model';
info.MatPath{8} = 'revision';
info.MatPath{9} = 'serial';
