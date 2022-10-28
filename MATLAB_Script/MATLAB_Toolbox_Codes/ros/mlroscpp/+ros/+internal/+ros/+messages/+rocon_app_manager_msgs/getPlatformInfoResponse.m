function [data, info] = getPlatformInfoResponse
%GetPlatformInfo gives an empty data for rocon_app_manager_msgs/GetPlatformInfoResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rocon_app_manager_msgs/GetPlatformInfoResponse';
[data.PlatformInfo, info.PlatformInfo] = ros.internal.ros.messages.rocon_app_manager_msgs.platformInfo;
info.PlatformInfo.MLdataType = 'struct';
info.MessageType = 'rocon_app_manager_msgs/GetPlatformInfoResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,20);
info.MatPath{1} = 'platform_info';
info.MatPath{2} = 'platform_info.PLATFORM_ANY';
info.MatPath{3} = 'platform_info.PLATFORM_LINUX';
info.MatPath{4} = 'platform_info.PLATFORM_WINDOZE';
info.MatPath{5} = 'platform_info.PLATFORM_ANDROID';
info.MatPath{6} = 'platform_info.SYSTEM_CUSTOM';
info.MatPath{7} = 'platform_info.SYSTEM_ROS';
info.MatPath{8} = 'platform_info.SYSTEM_OPROS';
info.MatPath{9} = 'platform_info.ROBOT_ANY';
info.MatPath{10} = 'platform_info.ROBOT_PC';
info.MatPath{11} = 'platform_info.ROBOT_ROBOSEM';
info.MatPath{12} = 'platform_info.ROBOT_KOBUKI';
info.MatPath{13} = 'platform_info.ROBOT_TURTLEBOT';
info.MatPath{14} = 'platform_info.platform';
info.MatPath{15} = 'platform_info.system';
info.MatPath{16} = 'platform_info.robot';
info.MatPath{17} = 'platform_info.name';
info.MatPath{18} = 'platform_info.icon';
info.MatPath{19} = 'platform_info.icon.format';
info.MatPath{20} = 'platform_info.icon.data';
