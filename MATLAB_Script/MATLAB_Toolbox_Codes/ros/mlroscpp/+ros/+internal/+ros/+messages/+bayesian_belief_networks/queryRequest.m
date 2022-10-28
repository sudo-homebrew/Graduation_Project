function [data, info] = queryRequest
%Query gives an empty data for bayesian_belief_networks/QueryRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'bayesian_belief_networks/QueryRequest';
[data.Query, info.Query] = ros.internal.ros.messages.bayesian_belief_networks.observation;
info.Query.MLdataType = 'struct';
info.Query.MaxLen = NaN;
info.Query.MinLen = 0;
data.Query = data.Query([],1);
info.MessageType = 'bayesian_belief_networks/QueryRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'query';
info.MatPath{2} = 'query.node';
info.MatPath{3} = 'query.evidence';
