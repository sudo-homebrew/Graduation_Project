function [data, info] = movingEdgeSite
%MovingEdgeSite gives an empty data for visp_tracker/MovingEdgeSite

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'visp_tracker/MovingEdgeSite';
[data.X, info.X] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Y, info.Y] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Suppress, info.Suppress] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'visp_tracker/MovingEdgeSite';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'x';
info.MatPath{2} = 'y';
info.MatPath{3} = 'suppress';
