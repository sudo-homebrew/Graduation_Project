function [data, info] = rawCamCal
%RawCamCal gives an empty data for multisense_ros/RawCamCal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'multisense_ros/RawCamCal';
[data.LeftM, info.LeftM] = ros.internal.ros.messages.ros.default_type('single',9);
[data.LeftD, info.LeftD] = ros.internal.ros.messages.ros.default_type('single',8);
[data.LeftR, info.LeftR] = ros.internal.ros.messages.ros.default_type('single',9);
[data.LeftP, info.LeftP] = ros.internal.ros.messages.ros.default_type('single',12);
[data.RightM, info.RightM] = ros.internal.ros.messages.ros.default_type('single',9);
[data.RightD, info.RightD] = ros.internal.ros.messages.ros.default_type('single',8);
[data.RightR, info.RightR] = ros.internal.ros.messages.ros.default_type('single',9);
[data.RightP, info.RightP] = ros.internal.ros.messages.ros.default_type('single',12);
info.MessageType = 'multisense_ros/RawCamCal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'left_M';
info.MatPath{2} = 'left_D';
info.MatPath{3} = 'left_R';
info.MatPath{4} = 'left_P';
info.MatPath{5} = 'right_M';
info.MatPath{6} = 'right_D';
info.MatPath{7} = 'right_R';
info.MatPath{8} = 'right_P';
