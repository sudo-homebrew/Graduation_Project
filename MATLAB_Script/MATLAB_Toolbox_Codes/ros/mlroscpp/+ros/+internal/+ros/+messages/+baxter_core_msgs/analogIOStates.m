function [data, info] = analogIOStates
%AnalogIOStates gives an empty data for baxter_core_msgs/AnalogIOStates

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'baxter_core_msgs/AnalogIOStates';
[data.Names, info.Names] = ros.internal.ros.messages.ros.char('string',NaN);
[data.States, info.States] = ros.internal.ros.messages.baxter_core_msgs.analogIOState;
info.States.MLdataType = 'struct';
info.States.MaxLen = NaN;
info.States.MinLen = 0;
data.States = data.States([],1);
info.MessageType = 'baxter_core_msgs/AnalogIOStates';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'names';
info.MatPath{2} = 'states';
info.MatPath{3} = 'states.timestamp';
info.MatPath{4} = 'states.timestamp.sec';
info.MatPath{5} = 'states.timestamp.nsec';
info.MatPath{6} = 'states.value';
info.MatPath{7} = 'states.isInputOnly';
