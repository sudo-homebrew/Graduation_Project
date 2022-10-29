function [data, info] = connectionsList
%ConnectionsList gives an empty data for rocon_std_msgs/ConnectionsList

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rocon_std_msgs/ConnectionsList';
[data.Connections, info.Connections] = ros.internal.ros.messages.rocon_std_msgs.connection;
info.Connections.MLdataType = 'struct';
info.Connections.MaxLen = NaN;
info.Connections.MinLen = 0;
data.Connections = data.Connections([],1);
info.MessageType = 'rocon_std_msgs/ConnectionsList';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,13);
info.MatPath{1} = 'connections';
info.MatPath{2} = 'connections.PUBLISHER';
info.MatPath{3} = 'connections.SUBSCRIBER';
info.MatPath{4} = 'connections.SERVICE';
info.MatPath{5} = 'connections.ACTION_CLIENT';
info.MatPath{6} = 'connections.ACTION_SERVER';
info.MatPath{7} = 'connections.INVALID';
info.MatPath{8} = 'connections.type';
info.MatPath{9} = 'connections.name';
info.MatPath{10} = 'connections.node';
info.MatPath{11} = 'connections.type_msg';
info.MatPath{12} = 'connections.type_info';
info.MatPath{13} = 'connections.xmlrpc_uri';
