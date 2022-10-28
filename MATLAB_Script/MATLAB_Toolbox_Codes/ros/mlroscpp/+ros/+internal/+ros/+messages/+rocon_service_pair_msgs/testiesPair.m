function [data, info] = testiesPair
%TestiesPair gives an empty data for rocon_service_pair_msgs/TestiesPair

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rocon_service_pair_msgs/TestiesPair';
[data.PairRequest, info.PairRequest] = ros.internal.ros.messages.rocon_service_pair_msgs.testiesPairRequest;
info.PairRequest.MLdataType = 'struct';
[data.PairResponse, info.PairResponse] = ros.internal.ros.messages.rocon_service_pair_msgs.testiesPairResponse;
info.PairResponse.MLdataType = 'struct';
info.MessageType = 'rocon_service_pair_msgs/TestiesPair';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'pair_request';
info.MatPath{2} = 'pair_request.id';
info.MatPath{3} = 'pair_request.id.uuid';
info.MatPath{4} = 'pair_request.request';
info.MatPath{5} = 'pair_request.request.data';
info.MatPath{6} = 'pair_response';
info.MatPath{7} = 'pair_response.id';
info.MatPath{8} = 'pair_response.id.uuid';
info.MatPath{9} = 'pair_response.response';
info.MatPath{10} = 'pair_response.response.id';
info.MatPath{11} = 'pair_response.response.id.uuid';
info.MatPath{12} = 'pair_response.response.data';
