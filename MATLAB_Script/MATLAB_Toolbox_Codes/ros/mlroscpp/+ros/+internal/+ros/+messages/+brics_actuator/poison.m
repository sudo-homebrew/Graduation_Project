function [data, info] = poison
%Poison gives an empty data for brics_actuator/Poison

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'brics_actuator/Poison';
[data.Originator, info.Originator] = ros.internal.ros.messages.ros.char('string',0);
[data.Description, info.Description] = ros.internal.ros.messages.ros.char('string',0);
[data.Qos, info.Qos] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'brics_actuator/Poison';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'originator';
info.MatPath{2} = 'description';
info.MatPath{3} = 'qos';
