function [data, info] = detectObjectsRequest
%DetectObjects gives an empty data for cob_object_detection_msgs/DetectObjectsRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_object_detection_msgs/DetectObjectsRequest';
[data.ObjectName, info.ObjectName] = ros.internal.ros.messages.std_msgs.string;
info.ObjectName.MLdataType = 'struct';
[data.Roi, info.Roi] = ros.internal.ros.messages.sensor_msgs.regionOfInterest;
info.Roi.MLdataType = 'struct';
info.MessageType = 'cob_object_detection_msgs/DetectObjectsRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'object_name';
info.MatPath{2} = 'object_name.data';
info.MatPath{3} = 'roi';
info.MatPath{4} = 'roi.x_offset';
info.MatPath{5} = 'roi.y_offset';
info.MatPath{6} = 'roi.height';
info.MatPath{7} = 'roi.width';
info.MatPath{8} = 'roi.do_rectify';
