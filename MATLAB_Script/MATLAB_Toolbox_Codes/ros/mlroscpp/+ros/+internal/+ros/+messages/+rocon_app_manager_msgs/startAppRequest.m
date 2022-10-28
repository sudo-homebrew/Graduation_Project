function [data, info] = startAppRequest
%StartApp gives an empty data for rocon_app_manager_msgs/StartAppRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rocon_app_manager_msgs/StartAppRequest';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Remappings, info.Remappings] = ros.internal.ros.messages.rocon_app_manager_msgs.remapping;
info.Remappings.MLdataType = 'struct';
info.Remappings.MaxLen = NaN;
info.Remappings.MinLen = 0;
data.Remappings = data.Remappings([],1);
info.MessageType = 'rocon_app_manager_msgs/StartAppRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'name';
info.MatPath{2} = 'remappings';
info.MatPath{3} = 'remappings.remap_from';
info.MatPath{4} = 'remappings.remap_to';
