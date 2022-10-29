function [data, info] = getAvailableStatesResponse
%GetAvailableStates gives an empty data for lifecycle_msgs/GetAvailableStatesResponse

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'lifecycle_msgs/GetAvailableStatesResponse';
[data.available_states, info.available_states] = ros.internal.ros2.messages.lifecycle_msgs.state;
info.available_states.MLdataType = 'struct';
info.available_states.MaxLen = NaN;
info.available_states.MinLen = 0;
info.MessageType = 'lifecycle_msgs/GetAvailableStatesResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'available_states';
info.MatPath{2} = 'available_states.PRIMARY_STATE_UNKNOWN';
info.MatPath{3} = 'available_states.PRIMARY_STATE_UNCONFIGURED';
info.MatPath{4} = 'available_states.PRIMARY_STATE_INACTIVE';
info.MatPath{5} = 'available_states.PRIMARY_STATE_ACTIVE';
info.MatPath{6} = 'available_states.PRIMARY_STATE_FINALIZED';
info.MatPath{7} = 'available_states.TRANSITION_STATE_CONFIGURING';
info.MatPath{8} = 'available_states.TRANSITION_STATE_CLEANINGUP';
info.MatPath{9} = 'available_states.TRANSITION_STATE_SHUTTINGDOWN';
info.MatPath{10} = 'available_states.TRANSITION_STATE_ACTIVATING';
info.MatPath{11} = 'available_states.TRANSITION_STATE_DEACTIVATING';
info.MatPath{12} = 'available_states.TRANSITION_STATE_ERRORPROCESSING';
info.MatPath{13} = 'available_states.id';
info.MatPath{14} = 'available_states.label';
