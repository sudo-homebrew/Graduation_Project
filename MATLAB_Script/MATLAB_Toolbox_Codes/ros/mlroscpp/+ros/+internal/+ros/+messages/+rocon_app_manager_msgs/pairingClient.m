function [data, info] = pairingClient
%PairingClient gives an empty data for rocon_app_manager_msgs/PairingClient

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rocon_app_manager_msgs/PairingClient';
[data.ClientType, info.ClientType] = ros.internal.ros.messages.ros.char('string',0);
[data.ManagerData, info.ManagerData] = ros.internal.ros.messages.rocon_app_manager_msgs.keyValue;
info.ManagerData.MLdataType = 'struct';
info.ManagerData.MaxLen = NaN;
info.ManagerData.MinLen = 0;
data.ManagerData = data.ManagerData([],1);
[data.AppData, info.AppData] = ros.internal.ros.messages.rocon_app_manager_msgs.keyValue;
info.AppData.MLdataType = 'struct';
info.AppData.MaxLen = NaN;
info.AppData.MinLen = 0;
data.AppData = data.AppData([],1);
info.MessageType = 'rocon_app_manager_msgs/PairingClient';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'client_type';
info.MatPath{2} = 'manager_data';
info.MatPath{3} = 'manager_data.key';
info.MatPath{4} = 'manager_data.value';
info.MatPath{5} = 'app_data';
info.MatPath{6} = 'app_data.key';
info.MatPath{7} = 'app_data.value';
