function [data, info] = sequence
%Sequence gives an empty data for cob_light/Sequence

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_light/Sequence';
[data.Color, info.Color] = ros.internal.ros.messages.std_msgs.colorRGBA;
info.Color.MLdataType = 'struct';
[data.HoldTime, info.HoldTime] = ros.internal.ros.messages.ros.default_type('single',1);
[data.CrossTime, info.CrossTime] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'cob_light/Sequence';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'color';
info.MatPath{2} = 'color.r';
info.MatPath{3} = 'color.g';
info.MatPath{4} = 'color.b';
info.MatPath{5} = 'color.a';
info.MatPath{6} = 'hold_time';
info.MatPath{7} = 'cross_time';
