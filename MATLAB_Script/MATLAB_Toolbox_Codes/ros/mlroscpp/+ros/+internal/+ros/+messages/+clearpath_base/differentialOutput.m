function [data, info] = differentialOutput
%DifferentialOutput gives an empty data for clearpath_base/DifferentialOutput

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'clearpath_base/DifferentialOutput';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Left, info.Left] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Right, info.Right] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'clearpath_base/DifferentialOutput';
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
info.MatPath{7} = 'left';
info.MatPath{8} = 'right';
