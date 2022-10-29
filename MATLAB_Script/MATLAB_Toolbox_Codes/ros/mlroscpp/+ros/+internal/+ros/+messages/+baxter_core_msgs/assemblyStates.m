function [data, info] = assemblyStates
%AssemblyStates gives an empty data for baxter_core_msgs/AssemblyStates

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'baxter_core_msgs/AssemblyStates';
[data.Names, info.Names] = ros.internal.ros.messages.ros.char('string',NaN);
[data.States, info.States] = ros.internal.ros.messages.baxter_core_msgs.assemblyState;
info.States.MLdataType = 'struct';
info.States.MaxLen = NaN;
info.States.MinLen = 0;
data.States = data.States([],1);
info.MessageType = 'baxter_core_msgs/AssemblyStates';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,17);
info.MatPath{1} = 'names';
info.MatPath{2} = 'states';
info.MatPath{3} = 'states.ready';
info.MatPath{4} = 'states.enabled';
info.MatPath{5} = 'states.stopped';
info.MatPath{6} = 'states.error';
info.MatPath{7} = 'states.estop_button';
info.MatPath{8} = 'states.ESTOP_BUTTON_UNPRESSED';
info.MatPath{9} = 'states.ESTOP_BUTTON_PRESSED';
info.MatPath{10} = 'states.ESTOP_BUTTON_UNKNOWN';
info.MatPath{11} = 'states.ESTOP_BUTTON_RELEASED';
info.MatPath{12} = 'states.estop_source';
info.MatPath{13} = 'states.ESTOP_SOURCE_NONE';
info.MatPath{14} = 'states.ESTOP_SOURCE_USER';
info.MatPath{15} = 'states.ESTOP_SOURCE_UNKNOWN';
info.MatPath{16} = 'states.ESTOP_SOURCE_FAULT';
info.MatPath{17} = 'states.ESTOP_SOURCE_BRAIN';
