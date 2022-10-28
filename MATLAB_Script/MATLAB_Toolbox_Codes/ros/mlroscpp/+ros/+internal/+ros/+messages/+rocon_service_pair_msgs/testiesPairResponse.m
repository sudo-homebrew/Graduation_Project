function [data, info] = testiesPairResponse
%TestiesPairResponse gives an empty data for rocon_service_pair_msgs/TestiesPairResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rocon_service_pair_msgs/TestiesPairResponse';
[data.Id, info.Id] = ros.internal.ros.messages.uuid_msgs.uniqueID;
info.Id.MLdataType = 'struct';
[data.Response, info.Response] = ros.internal.ros.messages.rocon_service_pair_msgs.testiesResponse;
info.Response.MLdataType = 'struct';
info.MessageType = 'rocon_service_pair_msgs/TestiesPairResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'id';
info.MatPath{2} = 'id.uuid';
info.MatPath{3} = 'response';
info.MatPath{4} = 'response.id';
info.MatPath{5} = 'response.id.uuid';
info.MatPath{6} = 'response.data';
