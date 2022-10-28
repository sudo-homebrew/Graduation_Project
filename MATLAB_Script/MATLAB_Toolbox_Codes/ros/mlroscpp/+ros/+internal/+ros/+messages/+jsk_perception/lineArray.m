function [data, info] = lineArray
%LineArray gives an empty data for jsk_perception/LineArray

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_perception/LineArray';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Lines, info.Lines] = ros.internal.ros.messages.jsk_perception.line;
info.Lines.MLdataType = 'struct';
info.Lines.MaxLen = NaN;
info.Lines.MinLen = 0;
data.Lines = data.Lines([],1);
info.MessageType = 'jsk_perception/LineArray';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'lines';
info.MatPath{8} = 'lines.x1';
info.MatPath{9} = 'lines.y1';
info.MatPath{10} = 'lines.x2';
info.MatPath{11} = 'lines.y2';
