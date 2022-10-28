function [data, info] = shapeGoal
%ShapeGoal gives an empty data for turtle_actionlib/ShapeGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'turtle_actionlib/ShapeGoal';
[data.Edges, info.Edges] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Radius, info.Radius] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'turtle_actionlib/ShapeGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'edges';
info.MatPath{2} = 'radius';
