function [data, info] = analogIOState
%AnalogIOState gives an empty data for baxter_core_msgs/AnalogIOState

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'baxter_core_msgs/AnalogIOState';
[data.Timestamp, info.Timestamp] = ros.internal.ros.messages.ros.time;
info.Timestamp.MLdataType = 'struct';
[data.Value, info.Value] = ros.internal.ros.messages.ros.default_type('double',1);
[data.IsInputOnly, info.IsInputOnly] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'baxter_core_msgs/AnalogIOState';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'timestamp';
info.MatPath{2} = 'timestamp.sec';
info.MatPath{3} = 'timestamp.nsec';
info.MatPath{4} = 'value';
info.MatPath{5} = 'isInputOnly';
