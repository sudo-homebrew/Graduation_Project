function [data, info] = listControllersResponse
%ListControllers gives an empty data for controller_manager_msgs/ListControllersResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'controller_manager_msgs/ListControllersResponse';
[data.Controller, info.Controller] = ros.internal.ros.messages.controller_manager_msgs.controllerState;
info.Controller.MLdataType = 'struct';
info.Controller.MaxLen = NaN;
info.Controller.MinLen = 0;
data.Controller = data.Controller([],1);
info.MessageType = 'controller_manager_msgs/ListControllersResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'controller';
info.MatPath{2} = 'controller.name';
info.MatPath{3} = 'controller.state';
info.MatPath{4} = 'controller.type';
info.MatPath{5} = 'controller.claimed_resources';
info.MatPath{6} = 'controller.claimed_resources.hardware_interface';
info.MatPath{7} = 'controller.claimed_resources.resources';
