function [data, info] = appInstallationState
%AppInstallationState gives an empty data for app_manager/AppInstallationState

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'app_manager/AppInstallationState';
[data.InstalledApps, info.InstalledApps] = ros.internal.ros.messages.app_manager.exchangeApp;
info.InstalledApps.MLdataType = 'struct';
info.InstalledApps.MaxLen = NaN;
info.InstalledApps.MinLen = 0;
data.InstalledApps = data.InstalledApps([],1);
[data.AvailableApps, info.AvailableApps] = ros.internal.ros.messages.app_manager.exchangeApp;
info.AvailableApps.MLdataType = 'struct';
info.AvailableApps.MaxLen = NaN;
info.AvailableApps.MinLen = 0;
data.AvailableApps = data.AvailableApps([],1);
info.MessageType = 'app_manager/AppInstallationState';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,20);
info.MatPath{1} = 'installed_apps';
info.MatPath{2} = 'installed_apps.name';
info.MatPath{3} = 'installed_apps.display_name';
info.MatPath{4} = 'installed_apps.version';
info.MatPath{5} = 'installed_apps.latest_version';
info.MatPath{6} = 'installed_apps.description';
info.MatPath{7} = 'installed_apps.icon';
info.MatPath{8} = 'installed_apps.icon.format';
info.MatPath{9} = 'installed_apps.icon.data';
info.MatPath{10} = 'installed_apps.hidden';
info.MatPath{11} = 'available_apps';
info.MatPath{12} = 'available_apps.name';
info.MatPath{13} = 'available_apps.display_name';
info.MatPath{14} = 'available_apps.version';
info.MatPath{15} = 'available_apps.latest_version';
info.MatPath{16} = 'available_apps.description';
info.MatPath{17} = 'available_apps.icon';
info.MatPath{18} = 'available_apps.icon.format';
info.MatPath{19} = 'available_apps.icon.data';
info.MatPath{20} = 'available_apps.hidden';