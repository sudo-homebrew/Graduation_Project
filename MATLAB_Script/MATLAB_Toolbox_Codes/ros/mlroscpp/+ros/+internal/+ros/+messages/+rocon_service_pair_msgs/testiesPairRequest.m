function [data, info] = testiesPairRequest
%TestiesPairRequest gives an empty data for rocon_service_pair_msgs/TestiesPairRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rocon_service_pair_msgs/TestiesPairRequest';
[data.Id, info.Id] = ros.internal.ros.messages.uuid_msgs.uniqueID;
info.Id.MLdataType = 'struct';
[data.Request, info.Request] = ros.internal.ros.messages.rocon_service_pair_msgs.testiesRequest;
info.Request.MLdataType = 'struct';
info.MessageType = 'rocon_service_pair_msgs/TestiesPairRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'id';
info.MatPath{2} = 'id.uuid';
info.MatPath{3} = 'request';
info.MatPath{4} = 'request.data';
