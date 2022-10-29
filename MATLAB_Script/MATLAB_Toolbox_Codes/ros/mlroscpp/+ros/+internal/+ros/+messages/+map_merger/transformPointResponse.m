function [data, info] = transformPointResponse
%TransformPoint gives an empty data for map_merger/TransformPointResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'map_merger/TransformPointResponse';
[data.Point, info.Point] = ros.internal.ros.messages.adhoc_communication.mmPoint;
info.Point.MLdataType = 'struct';
info.MessageType = 'map_merger/TransformPointResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'point';
info.MatPath{2} = 'point.src_robot';
info.MatPath{3} = 'point.x';
info.MatPath{4} = 'point.y';
