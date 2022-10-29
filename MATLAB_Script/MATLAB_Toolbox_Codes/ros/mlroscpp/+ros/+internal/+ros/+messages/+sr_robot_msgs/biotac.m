function [data, info] = biotac
%Biotac gives an empty data for sr_robot_msgs/Biotac

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/Biotac';
[data.Pac0, info.Pac0] = ros.internal.ros.messages.ros.default_type('int16',1);
[data.Pac1, info.Pac1] = ros.internal.ros.messages.ros.default_type('int16',1);
[data.Pdc, info.Pdc] = ros.internal.ros.messages.ros.default_type('int16',1);
[data.Tac, info.Tac] = ros.internal.ros.messages.ros.default_type('int16',1);
[data.Tdc, info.Tdc] = ros.internal.ros.messages.ros.default_type('int16',1);
[data.Electrodes, info.Electrodes] = ros.internal.ros.messages.ros.default_type('int16',19);
info.MessageType = 'sr_robot_msgs/Biotac';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'pac0';
info.MatPath{2} = 'pac1';
info.MatPath{3} = 'pdc';
info.MatPath{4} = 'tac';
info.MatPath{5} = 'tdc';
info.MatPath{6} = 'electrodes';
