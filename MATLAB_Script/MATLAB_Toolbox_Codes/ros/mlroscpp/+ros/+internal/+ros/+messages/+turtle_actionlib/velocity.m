function [data, info] = velocity
%Velocity gives an empty data for turtle_actionlib/Velocity

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'turtle_actionlib/Velocity';
[data.Linear, info.Linear] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Angular, info.Angular] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'turtle_actionlib/Velocity';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'linear';
info.MatPath{2} = 'angular';
