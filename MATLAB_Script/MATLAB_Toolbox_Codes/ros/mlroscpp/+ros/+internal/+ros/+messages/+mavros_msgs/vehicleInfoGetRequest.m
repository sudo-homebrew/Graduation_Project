function [data, info] = vehicleInfoGetRequest
%VehicleInfoGet gives an empty data for mavros_msgs/VehicleInfoGetRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/VehicleInfoGetRequest';
[data.GETMYSYSID, info.GETMYSYSID] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.GETMYCOMPID, info.GETMYCOMPID] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.Sysid, info.Sysid] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Compid, info.Compid] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.GetAll, info.GetAll] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'mavros_msgs/VehicleInfoGetRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'GET_MY_SYSID';
info.MatPath{2} = 'GET_MY_COMPID';
info.MatPath{3} = 'sysid';
info.MatPath{4} = 'compid';
info.MatPath{5} = 'get_all';
