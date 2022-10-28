function [data, info] = tF2Error
%TF2Error gives an empty data for tf2_msgs/TF2Error

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'tf2_msgs/TF2Error';
[data.NOERROR, info.NOERROR] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.LOOKUPERROR, info.LOOKUPERROR] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.CONNECTIVITYERROR, info.CONNECTIVITYERROR] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.EXTRAPOLATIONERROR, info.EXTRAPOLATIONERROR] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.INVALIDARGUMENTERROR, info.INVALIDARGUMENTERROR] = ros.internal.ros.messages.ros.default_type('uint8',1, 4);
[data.TIMEOUTERROR, info.TIMEOUTERROR] = ros.internal.ros.messages.ros.default_type('uint8',1, 5);
[data.TRANSFORMERROR, info.TRANSFORMERROR] = ros.internal.ros.messages.ros.default_type('uint8',1, 6);
[data.Error, info.Error] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.ErrorString, info.ErrorString] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'tf2_msgs/TF2Error';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'NO_ERROR';
info.MatPath{2} = 'LOOKUP_ERROR';
info.MatPath{3} = 'CONNECTIVITY_ERROR';
info.MatPath{4} = 'EXTRAPOLATION_ERROR';
info.MatPath{5} = 'INVALID_ARGUMENT_ERROR';
info.MatPath{6} = 'TIMEOUT_ERROR';
info.MatPath{7} = 'TRANSFORM_ERROR';
info.MatPath{8} = 'error';
info.MatPath{9} = 'error_string';
