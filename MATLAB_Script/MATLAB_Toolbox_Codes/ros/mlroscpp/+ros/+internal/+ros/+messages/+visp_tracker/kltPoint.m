function [data, info] = kltPoint
%KltPoint gives an empty data for visp_tracker/KltPoint

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'visp_tracker/KltPoint';
[data.I, info.I] = ros.internal.ros.messages.ros.default_type('double',1);
[data.J, info.J] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Id, info.Id] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'visp_tracker/KltPoint';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'i';
info.MatPath{2} = 'j';
info.MatPath{3} = 'id';
