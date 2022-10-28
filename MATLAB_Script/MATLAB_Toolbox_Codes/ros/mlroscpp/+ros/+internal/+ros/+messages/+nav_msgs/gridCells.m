function [data, info] = gridCells
%GridCells gives an empty data for nav_msgs/GridCells

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'nav_msgs/GridCells';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.CellWidth, info.CellWidth] = ros.internal.ros.messages.ros.default_type('single',1);
[data.CellHeight, info.CellHeight] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Cells, info.Cells] = ros.internal.ros.messages.geometry_msgs.point;
info.Cells.MLdataType = 'struct';
info.Cells.MaxLen = NaN;
info.Cells.MinLen = 0;
data.Cells = data.Cells([],1);
info.MessageType = 'nav_msgs/GridCells';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'cell_width';
info.MatPath{8} = 'cell_height';
info.MatPath{9} = 'cells';
info.MatPath{10} = 'cells.x';
info.MatPath{11} = 'cells.y';
info.MatPath{12} = 'cells.z';
