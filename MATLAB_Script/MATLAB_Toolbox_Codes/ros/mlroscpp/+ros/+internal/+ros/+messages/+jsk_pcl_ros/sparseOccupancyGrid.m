function [data, info] = sparseOccupancyGrid
%SparseOccupancyGrid gives an empty data for jsk_pcl_ros/SparseOccupancyGrid

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_pcl_ros/SparseOccupancyGrid';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.OriginPose, info.OriginPose] = ros.internal.ros.messages.geometry_msgs.pose;
info.OriginPose.MLdataType = 'struct';
[data.Resolution, info.Resolution] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Columns, info.Columns] = ros.internal.ros.messages.jsk_pcl_ros.sparseOccupancyGridColumn;
info.Columns.MLdataType = 'struct';
info.Columns.MaxLen = NaN;
info.Columns.MinLen = 0;
data.Columns = data.Columns([],1);
info.MessageType = 'jsk_pcl_ros/SparseOccupancyGrid';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,22);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'origin_pose';
info.MatPath{8} = 'origin_pose.position';
info.MatPath{9} = 'origin_pose.position.x';
info.MatPath{10} = 'origin_pose.position.y';
info.MatPath{11} = 'origin_pose.position.z';
info.MatPath{12} = 'origin_pose.orientation';
info.MatPath{13} = 'origin_pose.orientation.x';
info.MatPath{14} = 'origin_pose.orientation.y';
info.MatPath{15} = 'origin_pose.orientation.z';
info.MatPath{16} = 'origin_pose.orientation.w';
info.MatPath{17} = 'resolution';
info.MatPath{18} = 'columns';
info.MatPath{19} = 'columns.column_index';
info.MatPath{20} = 'columns.cells';
info.MatPath{21} = 'columns.cells.row_index';
info.MatPath{22} = 'columns.cells.value';