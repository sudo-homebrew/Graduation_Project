function [data, info] = setObjectStateRequest
%SetObjectState gives an empty data for hector_worldmodel_msgs/SetObjectStateRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'hector_worldmodel_msgs/SetObjectStateRequest';
[data.ObjectId, info.ObjectId] = ros.internal.ros.messages.ros.char('string',0);
[data.NewState, info.NewState] = ros.internal.ros.messages.hector_worldmodel_msgs.objectState;
info.NewState.MLdataType = 'struct';
info.MessageType = 'hector_worldmodel_msgs/SetObjectStateRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'object_id';
info.MatPath{2} = 'new_state';
info.MatPath{3} = 'new_state.UNKNOWN';
info.MatPath{4} = 'new_state.PENDING';
info.MatPath{5} = 'new_state.ACTIVE';
info.MatPath{6} = 'new_state.INACTIVE';
info.MatPath{7} = 'new_state.CONFIRMED';
info.MatPath{8} = 'new_state.DISCARDED';
info.MatPath{9} = 'new_state.APPROACHING';
info.MatPath{10} = 'new_state.state';
