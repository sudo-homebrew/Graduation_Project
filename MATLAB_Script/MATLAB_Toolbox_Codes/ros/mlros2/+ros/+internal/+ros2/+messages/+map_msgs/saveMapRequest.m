function [data, info] = saveMapRequest
%SaveMap gives an empty data for map_msgs/SaveMapRequest

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'map_msgs/SaveMapRequest';
[data.filename, info.filename] = ros.internal.ros2.messages.std_msgs.string;
info.filename.MLdataType = 'struct';
info.MessageType = 'map_msgs/SaveMapRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'filename';
info.MatPath{2} = 'filename.data';
