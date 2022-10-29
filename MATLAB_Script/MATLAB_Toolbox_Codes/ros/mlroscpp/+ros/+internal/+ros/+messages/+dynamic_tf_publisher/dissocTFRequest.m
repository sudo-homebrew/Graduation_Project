function [data, info] = dissocTFRequest
%DissocTF gives an empty data for dynamic_tf_publisher/DissocTFRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'dynamic_tf_publisher/DissocTFRequest';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.FrameId, info.FrameId] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'dynamic_tf_publisher/DissocTFRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'frame_id';
