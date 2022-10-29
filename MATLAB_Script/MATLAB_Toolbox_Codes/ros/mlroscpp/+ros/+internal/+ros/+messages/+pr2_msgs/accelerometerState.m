function [data, info] = accelerometerState
%AccelerometerState gives an empty data for pr2_msgs/AccelerometerState

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_msgs/AccelerometerState';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Samples, info.Samples] = ros.internal.ros.messages.geometry_msgs.vector3;
info.Samples.MLdataType = 'struct';
info.Samples.MaxLen = NaN;
info.Samples.MinLen = 0;
data.Samples = data.Samples([],1);
info.MessageType = 'pr2_msgs/AccelerometerState';
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
info.MatPath{7} = 'samples';
info.MatPath{8} = 'samples.x';
info.MatPath{9} = 'samples.y';
info.MatPath{10} = 'samples.z';
