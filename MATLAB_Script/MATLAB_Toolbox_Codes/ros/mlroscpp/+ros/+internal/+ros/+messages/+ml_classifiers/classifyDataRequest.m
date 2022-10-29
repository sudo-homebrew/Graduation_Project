function [data, info] = classifyDataRequest
%ClassifyData gives an empty data for ml_classifiers/ClassifyDataRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ml_classifiers/ClassifyDataRequest';
[data.Identifier, info.Identifier] = ros.internal.ros.messages.ros.char('string',0);
[data.Data, info.Data] = ros.internal.ros.messages.ml_classifiers.classDataPoint;
info.Data.MLdataType = 'struct';
info.Data.MaxLen = NaN;
info.Data.MinLen = 0;
data.Data = data.Data([],1);
info.MessageType = 'ml_classifiers/ClassifyDataRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'identifier';
info.MatPath{2} = 'data';
info.MatPath{3} = 'data.target_class';
info.MatPath{4} = 'data.point';
