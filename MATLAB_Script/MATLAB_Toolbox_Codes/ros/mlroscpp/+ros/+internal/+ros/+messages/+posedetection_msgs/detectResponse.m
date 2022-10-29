function [data, info] = detectResponse
%Detect gives an empty data for posedetection_msgs/DetectResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'posedetection_msgs/DetectResponse';
[data.ObjectDetection, info.ObjectDetection] = ros.internal.ros.messages.posedetection_msgs.objectDetection;
info.ObjectDetection.MLdataType = 'struct';
info.MessageType = 'posedetection_msgs/DetectResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,20);
info.MatPath{1} = 'object_detection';
info.MatPath{2} = 'object_detection.header';
info.MatPath{3} = 'object_detection.header.seq';
info.MatPath{4} = 'object_detection.header.stamp';
info.MatPath{5} = 'object_detection.header.stamp.sec';
info.MatPath{6} = 'object_detection.header.stamp.nsec';
info.MatPath{7} = 'object_detection.header.frame_id';
info.MatPath{8} = 'object_detection.objects';
info.MatPath{9} = 'object_detection.objects.pose';
info.MatPath{10} = 'object_detection.objects.pose.position';
info.MatPath{11} = 'object_detection.objects.pose.position.x';
info.MatPath{12} = 'object_detection.objects.pose.position.y';
info.MatPath{13} = 'object_detection.objects.pose.position.z';
info.MatPath{14} = 'object_detection.objects.pose.orientation';
info.MatPath{15} = 'object_detection.objects.pose.orientation.x';
info.MatPath{16} = 'object_detection.objects.pose.orientation.y';
info.MatPath{17} = 'object_detection.objects.pose.orientation.z';
info.MatPath{18} = 'object_detection.objects.pose.orientation.w';
info.MatPath{19} = 'object_detection.objects.reliability';
info.MatPath{20} = 'object_detection.objects.type';
