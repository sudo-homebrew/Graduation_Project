function [data, info] = portControl
%PortControl gives an empty data for applanix_msgs/PortControl

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/PortControl';
[data.Transaction, info.Transaction] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.GroupsCount, info.GroupsCount] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.Groups, info.Groups] = ros.internal.ros.messages.applanix_msgs.outputGroup;
info.Groups.MLdataType = 'struct';
info.Groups.MaxLen = NaN;
info.Groups.MinLen = 0;
data.Groups = data.Groups([],1);
[data.Rate, info.Rate] = ros.internal.ros.messages.ros.default_type('uint16',1);
info.MessageType = 'applanix_msgs/PortControl';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'transaction';
info.MatPath{2} = 'groups_count';
info.MatPath{3} = 'groups';
info.MatPath{4} = 'groups.group';
info.MatPath{5} = 'rate';
