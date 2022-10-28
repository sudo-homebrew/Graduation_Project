function [data, info] = sparseOccupancyGridColumn
%SparseOccupancyGridColumn gives an empty data for jsk_pcl_ros/SparseOccupancyGridColumn

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_pcl_ros/SparseOccupancyGridColumn';
[data.ColumnIndex, info.ColumnIndex] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Cells, info.Cells] = ros.internal.ros.messages.jsk_pcl_ros.sparseOccupancyGridCell;
info.Cells.MLdataType = 'struct';
info.Cells.MaxLen = NaN;
info.Cells.MinLen = 0;
data.Cells = data.Cells([],1);
info.MessageType = 'jsk_pcl_ros/SparseOccupancyGridColumn';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'column_index';
info.MatPath{2} = 'cells';
info.MatPath{3} = 'cells.row_index';
info.MatPath{4} = 'cells.value';
