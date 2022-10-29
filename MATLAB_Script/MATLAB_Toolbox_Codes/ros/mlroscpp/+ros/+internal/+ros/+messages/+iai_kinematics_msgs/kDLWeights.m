function [data, info] = kDLWeights
%KDLWeights gives an empty data for iai_kinematics_msgs/KDLWeights

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'iai_kinematics_msgs/KDLWeights';
[data.INVALIDMODE, info.INVALIDMODE] = ros.internal.ros.messages.ros.default_type('int8',1, 0);
[data.SETTS, info.SETTS] = ros.internal.ros.messages.ros.default_type('int8',1, 1);
[data.SETJS, info.SETJS] = ros.internal.ros.messages.ros.default_type('int8',1, 2);
[data.SETLAMBDA, info.SETLAMBDA] = ros.internal.ros.messages.ros.default_type('int8',1, 4);
[data.SETTSJS, info.SETTSJS] = ros.internal.ros.messages.ros.default_type('int8',1, 3);
[data.Mode, info.Mode] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.WeightTs, info.WeightTs] = ros.internal.ros.messages.ros.default_type('double',36);
[data.WeightJs, info.WeightJs] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.Lambda, info.Lambda] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'iai_kinematics_msgs/KDLWeights';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'INVALID_MODE';
info.MatPath{2} = 'SET_TS';
info.MatPath{3} = 'SET_JS';
info.MatPath{4} = 'SET_LAMBDA';
info.MatPath{5} = 'SET_TS_JS';
info.MatPath{6} = 'mode';
info.MatPath{7} = 'weight_ts';
info.MatPath{8} = 'weight_js';
info.MatPath{9} = 'lambda';
