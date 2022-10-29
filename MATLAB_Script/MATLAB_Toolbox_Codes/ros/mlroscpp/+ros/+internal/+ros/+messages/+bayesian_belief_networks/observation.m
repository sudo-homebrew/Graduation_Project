function [data, info] = observation
%Observation gives an empty data for bayesian_belief_networks/Observation

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'bayesian_belief_networks/Observation';
[data.Node, info.Node] = ros.internal.ros.messages.ros.char('string',0);
[data.Evidence, info.Evidence] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'bayesian_belief_networks/Observation';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'node';
info.MatPath{2} = 'evidence';
