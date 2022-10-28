function [data, info] = drive
%Drive gives an empty data for create_node/Drive

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'create_node/Drive';
[data.Velocity, info.Velocity] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Radius, info.Radius] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'create_node/Drive';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'velocity';
info.MatPath{2} = 'radius';
