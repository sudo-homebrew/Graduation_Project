function [data, info] = boundingBoxArray
%BoundingBoxArray gives an empty data for jsk_pcl_ros/BoundingBoxArray

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_pcl_ros/BoundingBoxArray';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Boxes, info.Boxes] = ros.internal.ros.messages.jsk_pcl_ros.boundingBox;
info.Boxes.MLdataType = 'struct';
info.Boxes.MaxLen = NaN;
info.Boxes.MinLen = 0;
data.Boxes = data.Boxes([],1);
info.MessageType = 'jsk_pcl_ros/BoundingBoxArray';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,27);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'boxes';
info.MatPath{8} = 'boxes.header';
info.MatPath{9} = 'boxes.header.seq';
info.MatPath{10} = 'boxes.header.stamp';
info.MatPath{11} = 'boxes.header.stamp.sec';
info.MatPath{12} = 'boxes.header.stamp.nsec';
info.MatPath{13} = 'boxes.header.frame_id';
info.MatPath{14} = 'boxes.pose';
info.MatPath{15} = 'boxes.pose.position';
info.MatPath{16} = 'boxes.pose.position.x';
info.MatPath{17} = 'boxes.pose.position.y';
info.MatPath{18} = 'boxes.pose.position.z';
info.MatPath{19} = 'boxes.pose.orientation';
info.MatPath{20} = 'boxes.pose.orientation.x';
info.MatPath{21} = 'boxes.pose.orientation.y';
info.MatPath{22} = 'boxes.pose.orientation.z';
info.MatPath{23} = 'boxes.pose.orientation.w';
info.MatPath{24} = 'boxes.dimensions';
info.MatPath{25} = 'boxes.dimensions.x';
info.MatPath{26} = 'boxes.dimensions.y';
info.MatPath{27} = 'boxes.dimensions.z';
