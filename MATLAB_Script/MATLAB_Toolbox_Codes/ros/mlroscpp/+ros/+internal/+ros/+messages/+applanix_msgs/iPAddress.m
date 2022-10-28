function [data, info] = iPAddress
%IPAddress gives an empty data for applanix_msgs/IPAddress

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/IPAddress';
[data.Transaction, info.Transaction] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.Address, info.Address] = ros.internal.ros.messages.ros.default_type('uint8',4);
[data.SubnetMask, info.SubnetMask] = ros.internal.ros.messages.ros.default_type('uint8',4);
info.MessageType = 'applanix_msgs/IPAddress';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'transaction';
info.MatPath{2} = 'address';
info.MatPath{3} = 'subnet_mask';
