function [data, info] = statusResponse
%Status gives an empty data for rocon_app_manager_msgs/StatusResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rocon_app_manager_msgs/StatusResponse';
[data.ApplicationNamespace, info.ApplicationNamespace] = ros.internal.ros.messages.ros.char('string',0);
[data.RemoteController, info.RemoteController] = ros.internal.ros.messages.ros.char('string',0);
[data.ApplicationStatus, info.ApplicationStatus] = ros.internal.ros.messages.ros.char('string',0);
[data.Application, info.Application] = ros.internal.ros.messages.rocon_app_manager_msgs.app;
info.Application.MLdataType = 'struct';
info.MessageType = 'rocon_app_manager_msgs/StatusResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,20);
info.MatPath{1} = 'application_namespace';
info.MatPath{2} = 'remote_controller';
info.MatPath{3} = 'application_status';
info.MatPath{4} = 'application';
info.MatPath{5} = 'application.name';
info.MatPath{6} = 'application.display_name';
info.MatPath{7} = 'application.description';
info.MatPath{8} = 'application.platform';
info.MatPath{9} = 'application.status';
info.MatPath{10} = 'application.icon';
info.MatPath{11} = 'application.icon.format';
info.MatPath{12} = 'application.icon.data';
info.MatPath{13} = 'application.pairing_clients';
info.MatPath{14} = 'application.pairing_clients.client_type';
info.MatPath{15} = 'application.pairing_clients.manager_data';
info.MatPath{16} = 'application.pairing_clients.manager_data.key';
info.MatPath{17} = 'application.pairing_clients.manager_data.value';
info.MatPath{18} = 'application.pairing_clients.app_data';
info.MatPath{19} = 'application.pairing_clients.app_data.key';
info.MatPath{20} = 'application.pairing_clients.app_data.value';
