function [data, info] = app
%App gives an empty data for app_manager/App

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'app_manager/App';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.DisplayName, info.DisplayName] = ros.internal.ros.messages.ros.char('string',0);
[data.Icon, info.Icon] = ros.internal.ros.messages.app_manager.icon;
info.Icon.MLdataType = 'struct';
[data.ClientApps, info.ClientApps] = ros.internal.ros.messages.app_manager.clientApp;
info.ClientApps.MLdataType = 'struct';
info.ClientApps.MaxLen = NaN;
info.ClientApps.MinLen = 0;
data.ClientApps = data.ClientApps([],1);
info.MessageType = 'app_manager/App';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,13);
info.MatPath{1} = 'name';
info.MatPath{2} = 'display_name';
info.MatPath{3} = 'icon';
info.MatPath{4} = 'icon.format';
info.MatPath{5} = 'icon.data';
info.MatPath{6} = 'client_apps';
info.MatPath{7} = 'client_apps.client_type';
info.MatPath{8} = 'client_apps.manager_data';
info.MatPath{9} = 'client_apps.manager_data.key';
info.MatPath{10} = 'client_apps.manager_data.value';
info.MatPath{11} = 'client_apps.app_data';
info.MatPath{12} = 'client_apps.app_data.key';
info.MatPath{13} = 'client_apps.app_data.value';
