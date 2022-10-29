function [data, info] = lightModes
%LightModes gives an empty data for cob_light/LightModes

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_light/LightModes';
[data.NONE, info.NONE] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.STATIC, info.STATIC] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.FLASH, info.FLASH] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.BREATH, info.BREATH] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.BREATHCOLOR, info.BREATHCOLOR] = ros.internal.ros.messages.ros.default_type('uint8',1, 4);
[data.FADECOLOR, info.FADECOLOR] = ros.internal.ros.messages.ros.default_type('uint8',1, 5);
[data.SEQ, info.SEQ] = ros.internal.ros.messages.ros.default_type('uint8',1, 6);
[data.CIRCLECOLORS, info.CIRCLECOLORS] = ros.internal.ros.messages.ros.default_type('uint8',1, 7);
[data.SWEEP, info.SWEEP] = ros.internal.ros.messages.ros.default_type('uint8',1, 8);
[data.DISTAPPROX, info.DISTAPPROX] = ros.internal.ros.messages.ros.default_type('uint8',1, 9);
[data.GLOW, info.GLOW] = ros.internal.ros.messages.ros.default_type('uint8',1, 10);
[data.XMAS, info.XMAS] = ros.internal.ros.messages.ros.default_type('uint8',1, 11);
info.MessageType = 'cob_light/LightModes';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'NONE';
info.MatPath{2} = 'STATIC';
info.MatPath{3} = 'FLASH';
info.MatPath{4} = 'BREATH';
info.MatPath{5} = 'BREATH_COLOR';
info.MatPath{6} = 'FADE_COLOR';
info.MatPath{7} = 'SEQ';
info.MatPath{8} = 'CIRCLE_COLORS';
info.MatPath{9} = 'SWEEP';
info.MatPath{10} = 'DIST_APPROX';
info.MatPath{11} = 'GLOW';
info.MatPath{12} = 'XMAS';
