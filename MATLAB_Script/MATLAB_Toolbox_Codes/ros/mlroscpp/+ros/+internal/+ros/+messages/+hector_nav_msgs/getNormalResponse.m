function [data, info] = getNormalResponse
%GetNormal gives an empty data for hector_nav_msgs/GetNormalResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'hector_nav_msgs/GetNormalResponse';
[data.Normal, info.Normal] = ros.internal.ros.messages.geometry_msgs.vector3;
info.Normal.MLdataType = 'struct';
info.MessageType = 'hector_nav_msgs/GetNormalResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'normal';
info.MatPath{2} = 'normal.x';
info.MatPath{3} = 'normal.y';
info.MatPath{4} = 'normal.z';
