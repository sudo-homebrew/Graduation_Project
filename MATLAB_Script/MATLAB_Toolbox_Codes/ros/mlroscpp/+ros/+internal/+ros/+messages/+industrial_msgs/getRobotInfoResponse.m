function [data, info] = getRobotInfoResponse
%GetRobotInfo gives an empty data for industrial_msgs/GetRobotInfoResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'industrial_msgs/GetRobotInfoResponse';
[data.Controller, info.Controller] = ros.internal.ros.messages.industrial_msgs.deviceInfo;
info.Controller.MLdataType = 'struct';
[data.Robots, info.Robots] = ros.internal.ros.messages.industrial_msgs.deviceInfo;
info.Robots.MLdataType = 'struct';
info.Robots.MaxLen = NaN;
info.Robots.MinLen = 0;
data.Robots = data.Robots([],1);
[data.Code, info.Code] = ros.internal.ros.messages.industrial_msgs.serviceReturnCode;
info.Code.MLdataType = 'struct';
info.MessageType = 'industrial_msgs/GetRobotInfoResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,16);
info.MatPath{1} = 'controller';
info.MatPath{2} = 'controller.model';
info.MatPath{3} = 'controller.serial_number';
info.MatPath{4} = 'controller.hw_version';
info.MatPath{5} = 'controller.sw_version';
info.MatPath{6} = 'controller.address';
info.MatPath{7} = 'robots';
info.MatPath{8} = 'robots.model';
info.MatPath{9} = 'robots.serial_number';
info.MatPath{10} = 'robots.hw_version';
info.MatPath{11} = 'robots.sw_version';
info.MatPath{12} = 'robots.address';
info.MatPath{13} = 'code';
info.MatPath{14} = 'code.val';
info.MatPath{15} = 'code.SUCCESS';
info.MatPath{16} = 'code.FAILURE';
