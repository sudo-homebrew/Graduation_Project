function [data, info] = addThermalSourceRequest
%AddThermalSource gives an empty data for stdr_msgs/AddThermalSourceRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'stdr_msgs/AddThermalSourceRequest';
[data.NewSource, info.NewSource] = ros.internal.ros.messages.stdr_msgs.thermalSource;
info.NewSource.MLdataType = 'struct';
info.MessageType = 'stdr_msgs/AddThermalSourceRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'newSource';
info.MatPath{2} = 'newSource.id';
info.MatPath{3} = 'newSource.degrees';
info.MatPath{4} = 'newSource.pose';
info.MatPath{5} = 'newSource.pose.x';
info.MatPath{6} = 'newSource.pose.y';
info.MatPath{7} = 'newSource.pose.theta';
