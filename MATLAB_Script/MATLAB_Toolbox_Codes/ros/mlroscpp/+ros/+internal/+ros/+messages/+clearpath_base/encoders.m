function [data, info] = encoders
%Encoders gives an empty data for clearpath_base/Encoders

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'clearpath_base/Encoders';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Encoders_, info.Encoders_] = ros.internal.ros.messages.clearpath_base.encoder;
info.Encoders_.MLdataType = 'struct';
info.Encoders_.MaxLen = NaN;
info.Encoders_.MinLen = 0;
data.Encoders_ = data.Encoders_([],1);
info.MessageType = 'clearpath_base/Encoders';
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
info.MatPath{7} = 'encoders';
info.MatPath{8} = 'encoders.travel';
info.MatPath{9} = 'encoders.speed';
