function [data, info] = constants
%Constants gives an empty data for bond/Constants

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'bond/Constants';
[data.DEADPUBLISHPERIOD, info.DEADPUBLISHPERIOD] = ros.internal.ros.messages.ros.default_type('single',1, 0.0500000007450581);
[data.DEFAULTCONNECTTIMEOUT, info.DEFAULTCONNECTTIMEOUT] = ros.internal.ros.messages.ros.default_type('single',1, 10);
[data.DEFAULTHEARTBEATTIMEOUT, info.DEFAULTHEARTBEATTIMEOUT] = ros.internal.ros.messages.ros.default_type('single',1, 4);
[data.DEFAULTDISCONNECTTIMEOUT, info.DEFAULTDISCONNECTTIMEOUT] = ros.internal.ros.messages.ros.default_type('single',1, 2);
[data.DEFAULTHEARTBEATPERIOD, info.DEFAULTHEARTBEATPERIOD] = ros.internal.ros.messages.ros.default_type('single',1, 1);
[data.DISABLEHEARTBEATTIMEOUTPARAM, info.DISABLEHEARTBEATTIMEOUTPARAM] = ros.internal.ros.messages.ros.char('string',0);
[data.DISABLEHEARTBEATTIMEOUTPARAM, info.DISABLEHEARTBEATTIMEOUTPARAM] = ros.internal.ros.messages.ros.char('string',1,'/bond_disable_heartbeat_timeout');
info.MessageType = 'bond/Constants';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'DEAD_PUBLISH_PERIOD';
info.MatPath{2} = 'DEFAULT_CONNECT_TIMEOUT';
info.MatPath{3} = 'DEFAULT_HEARTBEAT_TIMEOUT';
info.MatPath{4} = 'DEFAULT_DISCONNECT_TIMEOUT';
info.MatPath{5} = 'DEFAULT_HEARTBEAT_PERIOD';
info.MatPath{6} = 'DISABLE_HEARTBEAT_TIMEOUT_PARAM';
