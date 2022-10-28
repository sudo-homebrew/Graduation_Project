function [data, info] = saveRestoreControl
%SaveRestoreControl gives an empty data for applanix_msgs/SaveRestoreControl

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/SaveRestoreControl';
[data.Transaction, info.Transaction] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.CONTROLNONE, info.CONTROLNONE] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.CONTROLSAVENVM, info.CONTROLSAVENVM] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.CONTROLRESTORENVM, info.CONTROLRESTORENVM] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.CONTROLRESTOREFACTORY, info.CONTROLRESTOREFACTORY] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.Control, info.Control] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'applanix_msgs/SaveRestoreControl';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'transaction';
info.MatPath{2} = 'CONTROL_NONE';
info.MatPath{3} = 'CONTROL_SAVE_NVM';
info.MatPath{4} = 'CONTROL_RESTORE_NVM';
info.MatPath{5} = 'CONTROL_RESTORE_FACTORY';
info.MatPath{6} = 'control';
