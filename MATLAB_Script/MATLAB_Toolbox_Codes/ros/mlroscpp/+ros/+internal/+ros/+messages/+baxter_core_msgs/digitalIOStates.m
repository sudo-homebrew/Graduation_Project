function [data, info] = digitalIOStates
%DigitalIOStates gives an empty data for baxter_core_msgs/DigitalIOStates

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'baxter_core_msgs/DigitalIOStates';
[data.Names, info.Names] = ros.internal.ros.messages.ros.char('string',NaN);
[data.States, info.States] = ros.internal.ros.messages.baxter_core_msgs.digitalIOState;
info.States.MLdataType = 'struct';
info.States.MaxLen = NaN;
info.States.MinLen = 0;
data.States = data.States([],1);
info.MessageType = 'baxter_core_msgs/DigitalIOStates';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'names';
info.MatPath{2} = 'states';
info.MatPath{3} = 'states.state';
info.MatPath{4} = 'states.isInputOnly';
info.MatPath{5} = 'states.OFF';
info.MatPath{6} = 'states.ON';
info.MatPath{7} = 'states.PRESSED';
info.MatPath{8} = 'states.UNPRESSED';
