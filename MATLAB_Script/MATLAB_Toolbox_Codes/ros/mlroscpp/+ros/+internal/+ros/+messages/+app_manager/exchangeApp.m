function [data, info] = exchangeApp
%ExchangeApp gives an empty data for app_manager/ExchangeApp

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'app_manager/ExchangeApp';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.DisplayName, info.DisplayName] = ros.internal.ros.messages.ros.char('string',0);
[data.Version, info.Version] = ros.internal.ros.messages.ros.char('string',0);
[data.LatestVersion, info.LatestVersion] = ros.internal.ros.messages.ros.char('string',0);
[data.Description, info.Description] = ros.internal.ros.messages.ros.char('string',0);
[data.Icon, info.Icon] = ros.internal.ros.messages.app_manager.icon;
info.Icon.MLdataType = 'struct';
[data.Hidden, info.Hidden] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'app_manager/ExchangeApp';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'name';
info.MatPath{2} = 'display_name';
info.MatPath{3} = 'version';
info.MatPath{4} = 'latest_version';
info.MatPath{5} = 'description';
info.MatPath{6} = 'icon';
info.MatPath{7} = 'icon.format';
info.MatPath{8} = 'icon.data';
info.MatPath{9} = 'hidden';
