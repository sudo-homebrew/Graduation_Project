function [data, info] = gridCells
%GridCells gives an empty data for nav_msgs/GridCells

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'nav_msgs/GridCells';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.cell_width, info.cell_width] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
[data.cell_height, info.cell_height] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
[data.cells, info.cells] = ros.internal.ros2.messages.geometry_msgs.point;
info.cells.MLdataType = 'struct';
info.cells.MaxLen = NaN;
info.cells.MinLen = 0;
info.MessageType = 'nav_msgs/GridCells';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'cell_width';
info.MatPath{7} = 'cell_height';
info.MatPath{8} = 'cells';
info.MatPath{9} = 'cells.x';
info.MatPath{10} = 'cells.y';
info.MatPath{11} = 'cells.z';
