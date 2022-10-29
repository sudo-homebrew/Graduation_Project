function [data, info] = cmd
%cmd gives an empty data for nav2d_operator/cmd

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'nav2d_operator/cmd';
[data.Velocity, info.Velocity] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Turn, info.Turn] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Mode, info.Mode] = ros.internal.ros.messages.ros.default_type('int8',1);
info.MessageType = 'nav2d_operator/cmd';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'Velocity';
info.MatPath{2} = 'Turn';
info.MatPath{3} = 'Mode';
