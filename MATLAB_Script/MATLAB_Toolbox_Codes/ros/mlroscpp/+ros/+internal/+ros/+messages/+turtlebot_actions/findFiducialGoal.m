function [data, info] = findFiducialGoal
%FindFiducialGoal gives an empty data for turtlebot_actions/FindFiducialGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'turtlebot_actions/FindFiducialGoal';
[data.CHESSBOARD, info.CHESSBOARD] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.CIRCLESGRID, info.CIRCLESGRID] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.ASYMMETRICCIRCLESGRID, info.ASYMMETRICCIRCLESGRID] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.CameraName, info.CameraName] = ros.internal.ros.messages.ros.char('string',0);
[data.PatternWidth, info.PatternWidth] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.PatternHeight, info.PatternHeight] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.PatternSize, info.PatternSize] = ros.internal.ros.messages.ros.default_type('single',1);
[data.PatternType, info.PatternType] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'turtlebot_actions/FindFiducialGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'CHESSBOARD';
info.MatPath{2} = 'CIRCLES_GRID';
info.MatPath{3} = 'ASYMMETRIC_CIRCLES_GRID';
info.MatPath{4} = 'camera_name';
info.MatPath{5} = 'pattern_width';
info.MatPath{6} = 'pattern_height';
info.MatPath{7} = 'pattern_size';
info.MatPath{8} = 'pattern_type';
