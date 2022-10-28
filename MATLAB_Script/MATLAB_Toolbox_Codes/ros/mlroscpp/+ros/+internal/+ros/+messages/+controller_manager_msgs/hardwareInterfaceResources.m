function [data, info] = hardwareInterfaceResources
%HardwareInterfaceResources gives an empty data for controller_manager_msgs/HardwareInterfaceResources

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'controller_manager_msgs/HardwareInterfaceResources';
[data.HardwareInterface, info.HardwareInterface] = ros.internal.ros.messages.ros.char('string',0);
[data.Resources, info.Resources] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'controller_manager_msgs/HardwareInterfaceResources';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'hardware_interface';
info.MatPath{2} = 'resources';
