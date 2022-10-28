function [data, info] = header
%Header gives an empty data for std_msgs/Header

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'std_msgs/Header';
[data.Seq, info.Seq] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Stamp, info.Stamp] = ros.internal.ros.messages.ros.time;
info.Stamp.MLdataType = 'struct';
[data.FrameId, info.FrameId] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'std_msgs/Header';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'seq';
info.MatPath{2} = 'stamp';
info.MatPath{3} = 'stamp.sec';
info.MatPath{4} = 'stamp.nsec';
info.MatPath{5} = 'frame_id';
