function [data, info] = sparseOccupancyGridCell
%SparseOccupancyGridCell gives an empty data for jsk_pcl_ros/SparseOccupancyGridCell

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_pcl_ros/SparseOccupancyGridCell';
[data.RowIndex, info.RowIndex] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Value, info.Value] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'jsk_pcl_ros/SparseOccupancyGridCell';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'row_index';
info.MatPath{2} = 'value';
