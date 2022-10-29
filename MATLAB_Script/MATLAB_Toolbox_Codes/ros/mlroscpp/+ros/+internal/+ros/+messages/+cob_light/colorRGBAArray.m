function [data, info] = colorRGBAArray
%ColorRGBAArray gives an empty data for cob_light/ColorRGBAArray

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_light/ColorRGBAArray';
[data.Colors, info.Colors] = ros.internal.ros.messages.std_msgs.colorRGBA;
info.Colors.MLdataType = 'struct';
info.Colors.MaxLen = NaN;
info.Colors.MinLen = 0;
data.Colors = data.Colors([],1);
info.MessageType = 'cob_light/ColorRGBAArray';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'colors';
info.MatPath{2} = 'colors.r';
info.MatPath{3} = 'colors.g';
info.MatPath{4} = 'colors.b';
info.MatPath{5} = 'colors.a';
