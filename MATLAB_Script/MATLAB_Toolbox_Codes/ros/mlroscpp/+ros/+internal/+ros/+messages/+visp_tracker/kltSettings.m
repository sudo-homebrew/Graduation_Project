function [data, info] = kltSettings
%KltSettings gives an empty data for visp_tracker/KltSettings

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'visp_tracker/KltSettings';
[data.MaxFeatures, info.MaxFeatures] = ros.internal.ros.messages.ros.default_type('int64',1);
[data.WindowSize, info.WindowSize] = ros.internal.ros.messages.ros.default_type('int64',1);
[data.Quality, info.Quality] = ros.internal.ros.messages.ros.default_type('double',1);
[data.MinDistance, info.MinDistance] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Harris, info.Harris] = ros.internal.ros.messages.ros.default_type('double',1);
[data.SizeBlock, info.SizeBlock] = ros.internal.ros.messages.ros.default_type('int64',1);
[data.PyramidLvl, info.PyramidLvl] = ros.internal.ros.messages.ros.default_type('int64',1);
[data.MaskBorder, info.MaskBorder] = ros.internal.ros.messages.ros.default_type('int64',1);
info.MessageType = 'visp_tracker/KltSettings';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'max_features';
info.MatPath{2} = 'window_size';
info.MatPath{3} = 'quality';
info.MatPath{4} = 'min_distance';
info.MatPath{5} = 'harris';
info.MatPath{6} = 'size_block';
info.MatPath{7} = 'pyramid_lvl';
info.MatPath{8} = 'mask_border';
