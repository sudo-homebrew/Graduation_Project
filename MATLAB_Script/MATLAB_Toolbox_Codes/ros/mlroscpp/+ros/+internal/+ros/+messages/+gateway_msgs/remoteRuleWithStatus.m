function [data, info] = remoteRuleWithStatus
%RemoteRuleWithStatus gives an empty data for gateway_msgs/RemoteRuleWithStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gateway_msgs/RemoteRuleWithStatus';
[data.RemoteRule, info.RemoteRule] = ros.internal.ros.messages.gateway_msgs.remoteRule;
info.RemoteRule.MLdataType = 'struct';
[data.UNKNOWN, info.UNKNOWN] = ros.internal.ros.messages.ros.char('string',0);
[data.UNKNOWN, info.UNKNOWN] = ros.internal.ros.messages.ros.char('string',1,'unknown');
[data.PENDING, info.PENDING] = ros.internal.ros.messages.ros.char('string',0);
[data.PENDING, info.PENDING] = ros.internal.ros.messages.ros.char('string',1,'pending');
[data.ACCEPTED, info.ACCEPTED] = ros.internal.ros.messages.ros.char('string',0);
[data.ACCEPTED, info.ACCEPTED] = ros.internal.ros.messages.ros.char('string',1,'accepted');
[data.BLOCKED, info.BLOCKED] = ros.internal.ros.messages.ros.char('string',0);
[data.BLOCKED, info.BLOCKED] = ros.internal.ros.messages.ros.char('string',1,'blocked');
[data.RESEND, info.RESEND] = ros.internal.ros.messages.ros.char('string',0);
[data.RESEND, info.RESEND] = ros.internal.ros.messages.ros.char('string',1,'resend');
[data.Status, info.Status] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'gateway_msgs/RemoteRuleWithStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'remote_rule';
info.MatPath{2} = 'remote_rule.gateway';
info.MatPath{3} = 'remote_rule.rule';
info.MatPath{4} = 'remote_rule.rule.type';
info.MatPath{5} = 'remote_rule.rule.name';
info.MatPath{6} = 'remote_rule.rule.node';
info.MatPath{7} = 'UNKNOWN';
info.MatPath{8} = 'PENDING';
info.MatPath{9} = 'ACCEPTED';
info.MatPath{10} = 'BLOCKED';
info.MatPath{11} = 'RESEND';
info.MatPath{12} = 'status';
