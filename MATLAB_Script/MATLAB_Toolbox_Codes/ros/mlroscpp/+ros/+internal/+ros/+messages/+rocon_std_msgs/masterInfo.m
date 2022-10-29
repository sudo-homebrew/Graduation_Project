function [data, info] = masterInfo
%MasterInfo gives an empty data for rocon_std_msgs/MasterInfo

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rocon_std_msgs/MasterInfo';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.RoconUri, info.RoconUri] = ros.internal.ros.messages.ros.char('string',0);
[data.Description, info.Description] = ros.internal.ros.messages.ros.char('string',0);
[data.Icon, info.Icon] = ros.internal.ros.messages.rocon_std_msgs.icon;
info.Icon.MLdataType = 'struct';
[data.Version, info.Version] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'rocon_std_msgs/MasterInfo';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'name';
info.MatPath{2} = 'rocon_uri';
info.MatPath{3} = 'description';
info.MatPath{4} = 'icon';
info.MatPath{5} = 'icon.resource_name';
info.MatPath{6} = 'icon.format';
info.MatPath{7} = 'icon.data';
info.MatPath{8} = 'version';
