function [data, info] = uUID
%UUID gives an empty data for unique_identifier_msgs/UUID

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'unique_identifier_msgs/UUID';
[data.uuid, info.uuid] = ros.internal.ros2.messages.ros2.default_type('uint8',16,0);
info.MessageType = 'unique_identifier_msgs/UUID';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'uuid';
