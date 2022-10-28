function [data, info] = startControllerRequest
%StartController gives an empty data for dynamixel_controllers/StartControllerRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'dynamixel_controllers/StartControllerRequest';
[data.PortName, info.PortName] = ros.internal.ros.messages.ros.char('string',0);
[data.PackagePath, info.PackagePath] = ros.internal.ros.messages.ros.char('string',0);
[data.ModuleName, info.ModuleName] = ros.internal.ros.messages.ros.char('string',0);
[data.ClassName, info.ClassName] = ros.internal.ros.messages.ros.char('string',0);
[data.ControllerName, info.ControllerName] = ros.internal.ros.messages.ros.char('string',0);
[data.Dependencies, info.Dependencies] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'dynamixel_controllers/StartControllerRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'port_name';
info.MatPath{2} = 'package_path';
info.MatPath{3} = 'module_name';
info.MatPath{4} = 'class_name';
info.MatPath{5} = 'controller_name';
info.MatPath{6} = 'dependencies';
