function [data, info] = getTimeResponse
%GetTime gives an empty data for rosapi/GetTimeResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosapi/GetTimeResponse';
[data.Time, info.Time] = ros.internal.ros.messages.ros.time;
info.Time.MLdataType = 'struct';
info.MessageType = 'rosapi/GetTimeResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'time';
info.MatPath{2} = 'time.sec';
info.MatPath{3} = 'time.nsec';
