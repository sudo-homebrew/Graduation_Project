function [data, info] = errorCodes
%ErrorCodes gives an empty data for gateway_msgs/ErrorCodes

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gateway_msgs/ErrorCodes';
[data.SUCCESS, info.SUCCESS] = ros.internal.ros.messages.ros.default_type('int8',1, 0);
[data.NOHUBCONNECTION, info.NOHUBCONNECTION] = ros.internal.ros.messages.ros.default_type('int8',1, 1);
[data.HUBCONNECTIONALREADYEXISTS, info.HUBCONNECTIONALREADYEXISTS] = ros.internal.ros.messages.ros.default_type('int8',1, 11);
[data.HUBCONNECTIONUNRESOLVABLE, info.HUBCONNECTIONUNRESOLVABLE] = ros.internal.ros.messages.ros.default_type('int8',1, 12);
[data.HUBCONNECTIONBLACKLISTED, info.HUBCONNECTIONBLACKLISTED] = ros.internal.ros.messages.ros.default_type('int8',1, 13);
[data.HUBCONNECTIONFAILED, info.HUBCONNECTIONFAILED] = ros.internal.ros.messages.ros.default_type('int8',1, 14);
[data.HUBCONNECTIONNOTINNONEMPTYWHITELIST, info.HUBCONNECTIONNOTINNONEMPTYWHITELIST] = ros.internal.ros.messages.ros.default_type('int8',1, 15);
[data.HUBNAMENOTFOUND, info.HUBNAMENOTFOUND] = ros.internal.ros.messages.ros.default_type('int8',1, 16);
[data.HUBCONNECTIONLOST, info.HUBCONNECTIONLOST] = ros.internal.ros.messages.ros.default_type('int8',1, 17);
[data.HUBUNKNOWNERROR, info.HUBUNKNOWNERROR] = ros.internal.ros.messages.ros.default_type('int8',1, 19);
[data.FLIPRULEALREADYEXISTS, info.FLIPRULEALREADYEXISTS] = ros.internal.ros.messages.ros.default_type('int8',1, 22);
[data.FLIPPATTERNALREDYEXISTS, info.FLIPPATTERNALREDYEXISTS] = ros.internal.ros.messages.ros.default_type('int8',1, 23);
[data.FLIPREMOTEGATEWAYFIREWALLING, info.FLIPREMOTEGATEWAYFIREWALLING] = ros.internal.ros.messages.ros.default_type('int8',1, 24);
[data.ADVERTISEMENTEXISTS, info.ADVERTISEMENTEXISTS] = ros.internal.ros.messages.ros.default_type('int8',1, 31);
[data.ADVERTISEMENTNOTFOUND, info.ADVERTISEMENTNOTFOUND] = ros.internal.ros.messages.ros.default_type('int8',1, 32);
[data.UNKNOWNADVERTISEMENTERROR, info.UNKNOWNADVERTISEMENTERROR] = ros.internal.ros.messages.ros.default_type('int8',1, 39);
[data.PULLRULEALREADYEXISTS, info.PULLRULEALREADYEXISTS] = ros.internal.ros.messages.ros.default_type('int8',1, 41);
[data.REMOTEGATEWAYNOTVISIBLE, info.REMOTEGATEWAYNOTVISIBLE] = ros.internal.ros.messages.ros.default_type('int8',1, 51);
[data.REMOTEGATEWAYSELFISNOT, info.REMOTEGATEWAYSELFISNOT] = ros.internal.ros.messages.ros.default_type('int8',1, 52);
[data.REMOTEGATEWAYTARGETHASMULTIPLEMATCHES, info.REMOTEGATEWAYTARGETHASMULTIPLEMATCHES] = ros.internal.ros.messages.ros.default_type('int8',1, 53);
info.MessageType = 'gateway_msgs/ErrorCodes';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,20);
info.MatPath{1} = 'SUCCESS';
info.MatPath{2} = 'NO_HUB_CONNECTION';
info.MatPath{3} = 'HUB_CONNECTION_ALREADY_EXISTS';
info.MatPath{4} = 'HUB_CONNECTION_UNRESOLVABLE';
info.MatPath{5} = 'HUB_CONNECTION_BLACKLISTED';
info.MatPath{6} = 'HUB_CONNECTION_FAILED';
info.MatPath{7} = 'HUB_CONNECTION_NOT_IN_NONEMPTY_WHITELIST';
info.MatPath{8} = 'HUB_NAME_NOT_FOUND';
info.MatPath{9} = 'HUB_CONNECTION_LOST';
info.MatPath{10} = 'HUB_UNKNOWN_ERROR';
info.MatPath{11} = 'FLIP_RULE_ALREADY_EXISTS';
info.MatPath{12} = 'FLIP_PATTERN_ALREDY_EXISTS';
info.MatPath{13} = 'FLIP_REMOTE_GATEWAY_FIREWALLING';
info.MatPath{14} = 'ADVERTISEMENT_EXISTS';
info.MatPath{15} = 'ADVERTISEMENT_NOT_FOUND';
info.MatPath{16} = 'UNKNOWN_ADVERTISEMENT_ERROR';
info.MatPath{17} = 'PULL_RULE_ALREADY_EXISTS';
info.MatPath{18} = 'REMOTE_GATEWAY_NOT_VISIBLE';
info.MatPath{19} = 'REMOTE_GATEWAY_SELF_IS_NOT';
info.MatPath{20} = 'REMOTE_GATEWAY_TARGET_HAS_MULTIPLE_MATCHES';