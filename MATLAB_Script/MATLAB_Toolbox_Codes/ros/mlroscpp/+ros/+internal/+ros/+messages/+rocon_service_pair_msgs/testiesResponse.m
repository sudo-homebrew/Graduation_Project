function [data, info] = testiesResponse
%TestiesResponse gives an empty data for rocon_service_pair_msgs/TestiesResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rocon_service_pair_msgs/TestiesResponse';
[data.Id, info.Id] = ros.internal.ros.messages.uuid_msgs.uniqueID;
info.Id.MLdataType = 'struct';
[data.Data, info.Data] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'rocon_service_pair_msgs/TestiesResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'id';
info.MatPath{2} = 'id.uuid';
info.MatPath{3} = 'data';
