function [data, info] = messageDetailsResponse
%MessageDetails gives an empty data for rosapi/MessageDetailsResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosapi/MessageDetailsResponse';
[data.Typedefs, info.Typedefs] = ros.internal.ros.messages.rosapi.typeDef;
info.Typedefs.MLdataType = 'struct';
info.Typedefs.MaxLen = NaN;
info.Typedefs.MinLen = 0;
data.Typedefs = data.Typedefs([],1);
info.MessageType = 'rosapi/MessageDetailsResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'typedefs';
info.MatPath{2} = 'typedefs.type';
info.MatPath{3} = 'typedefs.fieldnames';
info.MatPath{4} = 'typedefs.fieldtypes';
info.MatPath{5} = 'typedefs.fieldarraylen';
info.MatPath{6} = 'typedefs.examples';
