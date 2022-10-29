function [data, info] = establishBondResponse
%EstablishBond gives an empty data for capabilities/EstablishBondResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'capabilities/EstablishBondResponse';
[data.BondId, info.BondId] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'capabilities/EstablishBondResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'bond_id';
