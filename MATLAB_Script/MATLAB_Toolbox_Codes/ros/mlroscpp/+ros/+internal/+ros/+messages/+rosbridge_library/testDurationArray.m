function [data, info] = testDurationArray
%TestDurationArray gives an empty data for rosbridge_library/TestDurationArray

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosbridge_library/TestDurationArray';
[data.Durations, info.Durations] = ros.internal.ros.messages.ros.duration;
info.Durations.MLdataType = 'struct';
info.Durations.MaxLen = NaN;
info.Durations.MinLen = 0;
data.Durations = data.Durations([],1);
info.MessageType = 'rosbridge_library/TestDurationArray';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'durations';
info.MatPath{2} = 'durations.sec';
info.MatPath{3} = 'durations.nsec';
