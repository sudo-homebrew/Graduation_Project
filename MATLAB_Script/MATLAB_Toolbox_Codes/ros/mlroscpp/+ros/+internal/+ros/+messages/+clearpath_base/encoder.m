function [data, info] = encoder
%Encoder gives an empty data for clearpath_base/Encoder

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'clearpath_base/Encoder';
[data.Travel, info.Travel] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Speed, info.Speed] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'clearpath_base/Encoder';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'travel';
info.MatPath{2} = 'speed';
