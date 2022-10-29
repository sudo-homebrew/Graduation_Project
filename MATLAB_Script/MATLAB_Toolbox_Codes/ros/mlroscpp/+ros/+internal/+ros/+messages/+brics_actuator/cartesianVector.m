function [data, info] = cartesianVector
%CartesianVector gives an empty data for brics_actuator/CartesianVector

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'brics_actuator/CartesianVector';
[data.Unit, info.Unit] = ros.internal.ros.messages.ros.char('string',0);
[data.X, info.X] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Y, info.Y] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Z, info.Z] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'brics_actuator/CartesianVector';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'unit';
info.MatPath{2} = 'x';
info.MatPath{3} = 'y';
info.MatPath{4} = 'z';
