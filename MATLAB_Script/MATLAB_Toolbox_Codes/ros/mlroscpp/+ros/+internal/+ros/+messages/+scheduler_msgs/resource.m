function [data, info] = resource
%Resource gives an empty data for scheduler_msgs/Resource

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'scheduler_msgs/Resource';
[data.Rapp, info.Rapp] = ros.internal.ros.messages.ros.char('string',0);
[data.Id, info.Id] = ros.internal.ros.messages.uuid_msgs.uniqueID;
info.Id.MLdataType = 'struct';
[data.Uri, info.Uri] = ros.internal.ros.messages.ros.char('string',0);
[data.Remappings, info.Remappings] = ros.internal.ros.messages.rocon_std_msgs.remapping;
info.Remappings.MLdataType = 'struct';
info.Remappings.MaxLen = NaN;
info.Remappings.MinLen = 0;
data.Remappings = data.Remappings([],1);
[data.Parameters, info.Parameters] = ros.internal.ros.messages.rocon_std_msgs.keyValue;
info.Parameters.MLdataType = 'struct';
info.Parameters.MaxLen = NaN;
info.Parameters.MinLen = 0;
data.Parameters = data.Parameters([],1);
info.MessageType = 'scheduler_msgs/Resource';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'rapp';
info.MatPath{2} = 'id';
info.MatPath{3} = 'id.uuid';
info.MatPath{4} = 'uri';
info.MatPath{5} = 'remappings';
info.MatPath{6} = 'remappings.remap_from';
info.MatPath{7} = 'remappings.remap_to';
info.MatPath{8} = 'parameters';
info.MatPath{9} = 'parameters.key';
info.MatPath{10} = 'parameters.value';
