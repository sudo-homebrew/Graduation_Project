function [data, info] = fkTestResponse
%FkTest gives an empty data for pr2_calibration_launch/FkTestResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_calibration_launch/FkTestResponse';
[data.Pos, info.Pos] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.Rot, info.Rot] = ros.internal.ros.messages.ros.default_type('double',NaN);
info.MessageType = 'pr2_calibration_launch/FkTestResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'pos';
info.MatPath{2} = 'rot';
