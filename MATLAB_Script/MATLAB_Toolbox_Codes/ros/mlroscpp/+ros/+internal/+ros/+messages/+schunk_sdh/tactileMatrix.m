function [data, info] = tactileMatrix
%TactileMatrix gives an empty data for schunk_sdh/TactileMatrix

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'schunk_sdh/TactileMatrix';
[data.MatrixId, info.MatrixId] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.CellsX, info.CellsX] = ros.internal.ros.messages.ros.default_type('int16',1);
[data.CellsY, info.CellsY] = ros.internal.ros.messages.ros.default_type('int16',1);
[data.TactileArray, info.TactileArray] = ros.internal.ros.messages.ros.default_type('int16',NaN);
info.MessageType = 'schunk_sdh/TactileMatrix';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'matrix_id';
info.MatPath{2} = 'cells_x';
info.MatPath{3} = 'cells_y';
info.MatPath{4} = 'tactile_array';
