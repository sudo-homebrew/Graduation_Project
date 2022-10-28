function [data, info] = controllerState
%ControllerState gives an empty data for controller_manager_msgs/ControllerState

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'controller_manager_msgs/ControllerState';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.State, info.State] = ros.internal.ros.messages.ros.char('string',0);
[data.Type, info.Type] = ros.internal.ros.messages.ros.char('string',0);
[data.ClaimedResources, info.ClaimedResources] = ros.internal.ros.messages.controller_manager_msgs.hardwareInterfaceResources;
info.ClaimedResources.MLdataType = 'struct';
info.ClaimedResources.MaxLen = NaN;
info.ClaimedResources.MinLen = 0;
data.ClaimedResources = data.ClaimedResources([],1);
info.MessageType = 'controller_manager_msgs/ControllerState';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'name';
info.MatPath{2} = 'state';
info.MatPath{3} = 'type';
info.MatPath{4} = 'claimed_resources';
info.MatPath{5} = 'claimed_resources.hardware_interface';
info.MatPath{6} = 'claimed_resources.resources';
