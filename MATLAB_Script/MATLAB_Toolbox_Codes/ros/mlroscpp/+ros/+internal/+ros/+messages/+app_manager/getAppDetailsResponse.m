function [data, info] = getAppDetailsResponse
%GetAppDetails gives an empty data for app_manager/GetAppDetailsResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'app_manager/GetAppDetailsResponse';
[data.App, info.App] = ros.internal.ros.messages.app_manager.exchangeApp;
info.App.MLdataType = 'struct';
info.MessageType = 'app_manager/GetAppDetailsResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'app';
info.MatPath{2} = 'app.name';
info.MatPath{3} = 'app.display_name';
info.MatPath{4} = 'app.version';
info.MatPath{5} = 'app.latest_version';
info.MatPath{6} = 'app.description';
info.MatPath{7} = 'app.icon';
info.MatPath{8} = 'app.icon.format';
info.MatPath{9} = 'app.icon.data';
info.MatPath{10} = 'app.hidden';
