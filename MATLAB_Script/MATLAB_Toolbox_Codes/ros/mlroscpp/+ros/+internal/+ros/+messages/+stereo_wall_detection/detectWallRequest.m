function [data, info] = detectWallRequest
%DetectWall gives an empty data for stereo_wall_detection/DetectWallRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'stereo_wall_detection/DetectWallRequest';
info.MessageType = 'stereo_wall_detection/DetectWallRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
