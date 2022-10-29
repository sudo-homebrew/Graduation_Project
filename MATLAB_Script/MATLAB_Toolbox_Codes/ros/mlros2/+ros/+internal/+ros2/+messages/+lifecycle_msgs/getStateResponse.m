function [data, info] = getStateResponse
%GetState gives an empty data for lifecycle_msgs/GetStateResponse

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'lifecycle_msgs/GetStateResponse';
[data.current_state, info.current_state] = ros.internal.ros2.messages.lifecycle_msgs.state;
info.current_state.MLdataType = 'struct';
info.MessageType = 'lifecycle_msgs/GetStateResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'current_state';
info.MatPath{2} = 'current_state.PRIMARY_STATE_UNKNOWN';
info.MatPath{3} = 'current_state.PRIMARY_STATE_UNCONFIGURED';
info.MatPath{4} = 'current_state.PRIMARY_STATE_INACTIVE';
info.MatPath{5} = 'current_state.PRIMARY_STATE_ACTIVE';
info.MatPath{6} = 'current_state.PRIMARY_STATE_FINALIZED';
info.MatPath{7} = 'current_state.TRANSITION_STATE_CONFIGURING';
info.MatPath{8} = 'current_state.TRANSITION_STATE_CLEANINGUP';
info.MatPath{9} = 'current_state.TRANSITION_STATE_SHUTTINGDOWN';
info.MatPath{10} = 'current_state.TRANSITION_STATE_ACTIVATING';
info.MatPath{11} = 'current_state.TRANSITION_STATE_DEACTIVATING';
info.MatPath{12} = 'current_state.TRANSITION_STATE_ERRORPROCESSING';
info.MatPath{13} = 'current_state.id';
info.MatPath{14} = 'current_state.label';
