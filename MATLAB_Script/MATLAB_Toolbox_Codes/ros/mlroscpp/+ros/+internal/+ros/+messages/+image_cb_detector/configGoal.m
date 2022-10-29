function [data, info] = configGoal
%ConfigGoal gives an empty data for image_cb_detector/ConfigGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'image_cb_detector/ConfigGoal';
[data.NumX, info.NumX] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.NumY, info.NumY] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.SpacingX, info.SpacingX] = ros.internal.ros.messages.ros.default_type('single',1);
[data.SpacingY, info.SpacingY] = ros.internal.ros.messages.ros.default_type('single',1);
[data.WidthScaling, info.WidthScaling] = ros.internal.ros.messages.ros.default_type('single',1);
[data.HeightScaling, info.HeightScaling] = ros.internal.ros.messages.ros.default_type('single',1);
[data.SubpixelWindow, info.SubpixelWindow] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.SubpixelZeroZone, info.SubpixelZeroZone] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'image_cb_detector/ConfigGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'num_x';
info.MatPath{2} = 'num_y';
info.MatPath{3} = 'spacing_x';
info.MatPath{4} = 'spacing_y';
info.MatPath{5} = 'width_scaling';
info.MatPath{6} = 'height_scaling';
info.MatPath{7} = 'subpixel_window';
info.MatPath{8} = 'subpixel_zero_zone';
