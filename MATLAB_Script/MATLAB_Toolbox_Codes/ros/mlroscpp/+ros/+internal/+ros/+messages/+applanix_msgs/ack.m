function [data, info] = ack
%Ack gives an empty data for applanix_msgs/Ack

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/Ack';
[data.Transaction, info.Transaction] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.Id, info.Id] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.RESPONSENOTAPPLICABLE, info.RESPONSENOTAPPLICABLE] = ros.internal.ros.messages.ros.default_type('uint16',1, 0);
[data.RESPONSEACCEPTED, info.RESPONSEACCEPTED] = ros.internal.ros.messages.ros.default_type('uint16',1, 1);
[data.RESPONSEACCEPTEDTOOLONG, info.RESPONSEACCEPTEDTOOLONG] = ros.internal.ros.messages.ros.default_type('uint16',1, 2);
[data.RESPONSEACCEPTEDTOOSHORT, info.RESPONSEACCEPTEDTOOSHORT] = ros.internal.ros.messages.ros.default_type('uint16',1, 3);
[data.RESPONSEPARAMERROR, info.RESPONSEPARAMERROR] = ros.internal.ros.messages.ros.default_type('uint16',1, 4);
[data.RESPONSENOTAPPLICABLEINCURRENTSTATE, info.RESPONSENOTAPPLICABLEINCURRENTSTATE] = ros.internal.ros.messages.ros.default_type('uint16',1, 5);
[data.RESPONSEDATANOTAVAILABLE, info.RESPONSEDATANOTAVAILABLE] = ros.internal.ros.messages.ros.default_type('uint16',1, 6);
[data.RESPONSEMESSAGESTARTERROR, info.RESPONSEMESSAGESTARTERROR] = ros.internal.ros.messages.ros.default_type('uint16',1, 7);
[data.RESPONSEMESSAGEENDERROR, info.RESPONSEMESSAGEENDERROR] = ros.internal.ros.messages.ros.default_type('uint16',1, 8);
[data.RESPONSEBYTECOUNTERROR, info.RESPONSEBYTECOUNTERROR] = ros.internal.ros.messages.ros.default_type('uint16',1, 9);
[data.RESPONSECHECKSUMERROR, info.RESPONSECHECKSUMERROR] = ros.internal.ros.messages.ros.default_type('uint16',1, 10);
[data.ResponseCode, info.ResponseCode] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.PARAMSNOCHANGE, info.PARAMSNOCHANGE] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.PARAMSCHANGE, info.PARAMSCHANGE] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.ParamsStatus, info.ParamsStatus] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.ErrorParameterName, info.ErrorParameterName] = ros.internal.ros.messages.ros.default_type('uint8',32);
info.MessageType = 'applanix_msgs/Ack';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,18);
info.MatPath{1} = 'transaction';
info.MatPath{2} = 'id';
info.MatPath{3} = 'RESPONSE_NOT_APPLICABLE';
info.MatPath{4} = 'RESPONSE_ACCEPTED';
info.MatPath{5} = 'RESPONSE_ACCEPTED_TOO_LONG';
info.MatPath{6} = 'RESPONSE_ACCEPTED_TOO_SHORT';
info.MatPath{7} = 'RESPONSE_PARAM_ERROR';
info.MatPath{8} = 'RESPONSE_NOT_APPLICABLE_IN_CURRENT_STATE';
info.MatPath{9} = 'RESPONSE_DATA_NOT_AVAILABLE';
info.MatPath{10} = 'RESPONSE_MESSAGE_START_ERROR';
info.MatPath{11} = 'RESPONSE_MESSAGE_END_ERROR';
info.MatPath{12} = 'RESPONSE_BYTE_COUNT_ERROR';
info.MatPath{13} = 'RESPONSE_CHECKSUM_ERROR';
info.MatPath{14} = 'response_code';
info.MatPath{15} = 'PARAMS_NO_CHANGE';
info.MatPath{16} = 'PARAMS_CHANGE';
info.MatPath{17} = 'params_status';
info.MatPath{18} = 'error_parameter_name';