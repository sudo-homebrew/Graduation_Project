function [data, info] = classDataPoint
%ClassDataPoint gives an empty data for ml_classifiers/ClassDataPoint

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ml_classifiers/ClassDataPoint';
[data.TargetClass, info.TargetClass] = ros.internal.ros.messages.ros.char('string',0);
[data.Point, info.Point] = ros.internal.ros.messages.ros.default_type('double',NaN);
info.MessageType = 'ml_classifiers/ClassDataPoint';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'target_class';
info.MatPath{2} = 'point';
