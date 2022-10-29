function [data, info] = getLoggersResponse
%GetLoggers gives an empty data for roscpp/GetLoggersResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'roscpp/GetLoggersResponse';
[data.Loggers, info.Loggers] = ros.internal.ros.messages.roscpp.logger;
info.Loggers.MLdataType = 'struct';
info.Loggers.MaxLen = NaN;
info.Loggers.MinLen = 0;
data.Loggers = data.Loggers([],1);
info.MessageType = 'roscpp/GetLoggersResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'loggers';
info.MatPath{2} = 'loggers.name';
info.MatPath{3} = 'loggers.level';
