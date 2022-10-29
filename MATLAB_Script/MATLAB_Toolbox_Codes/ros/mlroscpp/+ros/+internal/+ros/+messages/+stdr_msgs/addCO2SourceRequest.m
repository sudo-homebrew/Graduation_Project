function [data, info] = addCO2SourceRequest
%AddCO2Source gives an empty data for stdr_msgs/AddCO2SourceRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'stdr_msgs/AddCO2SourceRequest';
[data.NewSource, info.NewSource] = ros.internal.ros.messages.stdr_msgs.cO2Source;
info.NewSource.MLdataType = 'struct';
info.MessageType = 'stdr_msgs/AddCO2SourceRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'newSource';
info.MatPath{2} = 'newSource.id';
info.MatPath{3} = 'newSource.ppm';
info.MatPath{4} = 'newSource.pose';
info.MatPath{5} = 'newSource.pose.x';
info.MatPath{6} = 'newSource.pose.y';
info.MatPath{7} = 'newSource.pose.theta';
