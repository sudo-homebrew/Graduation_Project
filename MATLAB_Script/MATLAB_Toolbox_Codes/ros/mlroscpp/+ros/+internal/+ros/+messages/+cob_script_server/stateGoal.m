function [data, info] = stateGoal
%StateGoal gives an empty data for cob_script_server/StateGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_script_server/StateGoal';
[data.State, info.State] = ros.internal.ros.messages.cob_script_server.scriptState;
info.State.MLdataType = 'struct';
info.MessageType = 'cob_script_server/StateGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,18);
info.MatPath{1} = 'state';
info.MatPath{2} = 'state.header';
info.MatPath{3} = 'state.header.seq';
info.MatPath{4} = 'state.header.stamp';
info.MatPath{5} = 'state.header.stamp.sec';
info.MatPath{6} = 'state.header.stamp.nsec';
info.MatPath{7} = 'state.header.frame_id';
info.MatPath{8} = 'state.function_name';
info.MatPath{9} = 'state.component_name';
info.MatPath{10} = 'state.parameter_name';
info.MatPath{11} = 'state.full_graph_name';
info.MatPath{12} = 'state.UNKNOWN';
info.MatPath{13} = 'state.ACTIVE';
info.MatPath{14} = 'state.SUCCEEDED';
info.MatPath{15} = 'state.FAILED';
info.MatPath{16} = 'state.PAUSED';
info.MatPath{17} = 'state.state';
info.MatPath{18} = 'state.error_code';
