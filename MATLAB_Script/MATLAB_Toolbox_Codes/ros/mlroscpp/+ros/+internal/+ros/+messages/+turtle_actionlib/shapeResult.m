function [data, info] = shapeResult
%ShapeResult gives an empty data for turtle_actionlib/ShapeResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'turtle_actionlib/ShapeResult';
[data.InteriorAngle, info.InteriorAngle] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Apothem, info.Apothem] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'turtle_actionlib/ShapeResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'interior_angle';
info.MatPath{2} = 'apothem';
