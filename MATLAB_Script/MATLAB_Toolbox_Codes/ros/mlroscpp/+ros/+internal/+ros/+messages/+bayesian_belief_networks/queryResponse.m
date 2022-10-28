function [data, info] = queryResponse
%Query gives an empty data for bayesian_belief_networks/QueryResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'bayesian_belief_networks/QueryResponse';
[data.Results, info.Results] = ros.internal.ros.messages.bayesian_belief_networks.result;
info.Results.MLdataType = 'struct';
info.Results.MaxLen = NaN;
info.Results.MinLen = 0;
data.Results = data.Results([],1);
info.MessageType = 'bayesian_belief_networks/QueryResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'results';
info.MatPath{2} = 'results.node';
info.MatPath{3} = 'results.Value';
info.MatPath{4} = 'results.Marginal';
