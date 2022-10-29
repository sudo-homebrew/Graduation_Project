function [data, info] = position2DInt
%Position2DInt gives an empty data for base_local_planner/Position2DInt

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'base_local_planner/Position2DInt';
[data.X, info.X] = ros.internal.ros.messages.ros.default_type('int64',1);
[data.Y, info.Y] = ros.internal.ros.messages.ros.default_type('int64',1);
info.MessageType = 'base_local_planner/Position2DInt';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'x';
info.MatPath{2} = 'y';
