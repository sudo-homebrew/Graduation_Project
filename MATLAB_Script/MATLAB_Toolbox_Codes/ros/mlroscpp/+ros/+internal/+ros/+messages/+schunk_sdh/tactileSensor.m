function [data, info] = tactileSensor
%TactileSensor gives an empty data for schunk_sdh/TactileSensor

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'schunk_sdh/TactileSensor';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.TactileMatrix, info.TactileMatrix] = ros.internal.ros.messages.schunk_sdh.tactileMatrix;
info.TactileMatrix.MLdataType = 'struct';
info.TactileMatrix.MaxLen = NaN;
info.TactileMatrix.MinLen = 0;
data.TactileMatrix = data.TactileMatrix([],1);
info.MessageType = 'schunk_sdh/TactileSensor';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'tactile_matrix';
info.MatPath{8} = 'tactile_matrix.matrix_id';
info.MatPath{9} = 'tactile_matrix.cells_x';
info.MatPath{10} = 'tactile_matrix.cells_y';
info.MatPath{11} = 'tactile_matrix.tactile_array';
