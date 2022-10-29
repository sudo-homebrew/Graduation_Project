function [data, info] = getBoundedMapRequest
%GetBoundedMap gives an empty data for ethzasl_icp_mapper/GetBoundedMapRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ethzasl_icp_mapper/GetBoundedMapRequest';
[data.MapCenter, info.MapCenter] = ros.internal.ros.messages.geometry_msgs.pose;
info.MapCenter.MLdataType = 'struct';
[data.TopRightCorner, info.TopRightCorner] = ros.internal.ros.messages.geometry_msgs.point;
info.TopRightCorner.MLdataType = 'struct';
[data.BottomLeftCorner, info.BottomLeftCorner] = ros.internal.ros.messages.geometry_msgs.point;
info.BottomLeftCorner.MLdataType = 'struct';
info.MessageType = 'ethzasl_icp_mapper/GetBoundedMapRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,18);
info.MatPath{1} = 'mapCenter';
info.MatPath{2} = 'mapCenter.position';
info.MatPath{3} = 'mapCenter.position.x';
info.MatPath{4} = 'mapCenter.position.y';
info.MatPath{5} = 'mapCenter.position.z';
info.MatPath{6} = 'mapCenter.orientation';
info.MatPath{7} = 'mapCenter.orientation.x';
info.MatPath{8} = 'mapCenter.orientation.y';
info.MatPath{9} = 'mapCenter.orientation.z';
info.MatPath{10} = 'mapCenter.orientation.w';
info.MatPath{11} = 'topRightCorner';
info.MatPath{12} = 'topRightCorner.x';
info.MatPath{13} = 'topRightCorner.y';
info.MatPath{14} = 'topRightCorner.z';
info.MatPath{15} = 'bottomLeftCorner';
info.MatPath{16} = 'bottomLeftCorner.x';
info.MatPath{17} = 'bottomLeftCorner.y';
info.MatPath{18} = 'bottomLeftCorner.z';
