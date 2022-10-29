function [data, info] = testTimeArray
%TestTimeArray gives an empty data for rosbridge_library/TestTimeArray

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosbridge_library/TestTimeArray';
[data.Times, info.Times] = ros.internal.ros.messages.ros.time;
info.Times.MLdataType = 'struct';
info.Times.MaxLen = NaN;
info.Times.MinLen = 0;
data.Times = data.Times([],1);
info.MessageType = 'rosbridge_library/TestTimeArray';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'times';
info.MatPath{2} = 'times.sec';
info.MatPath{3} = 'times.nsec';
