function [data, info] = result
%Result gives an empty data for bayesian_belief_networks/Result

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'bayesian_belief_networks/Result';
[data.Node, info.Node] = ros.internal.ros.messages.ros.char('string',0);
[data.Value, info.Value] = ros.internal.ros.messages.ros.char('string',0);
[data.Marginal, info.Marginal] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'bayesian_belief_networks/Result';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'node';
info.MatPath{2} = 'Value';
info.MatPath{3} = 'Marginal';
