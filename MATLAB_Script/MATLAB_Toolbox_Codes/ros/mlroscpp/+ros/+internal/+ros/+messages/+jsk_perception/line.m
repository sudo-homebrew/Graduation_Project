function [data, info] = line
%Line gives an empty data for jsk_perception/Line

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_perception/Line';
[data.X1, info.X1] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Y1, info.Y1] = ros.internal.ros.messages.ros.default_type('double',1);
[data.X2, info.X2] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Y2, info.Y2] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'jsk_perception/Line';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'x1';
info.MatPath{2} = 'y1';
info.MatPath{3} = 'x2';
info.MatPath{4} = 'y2';
