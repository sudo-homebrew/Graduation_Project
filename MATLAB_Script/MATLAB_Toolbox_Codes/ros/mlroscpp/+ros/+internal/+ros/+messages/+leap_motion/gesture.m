function [data, info] = gesture
%Gesture gives an empty data for leap_motion/Gesture

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'leap_motion/Gesture';
[data.LmcGestureId, info.LmcGestureId] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.IsValid, info.IsValid] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.DurationUs, info.DurationUs] = ros.internal.ros.messages.ros.default_type('int64',1);
[data.DurationS, info.DurationS] = ros.internal.ros.messages.ros.default_type('single',1);
[data.GestureState, info.GestureState] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.GestureType, info.GestureType] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.ToString, info.ToString] = ros.internal.ros.messages.ros.char('string',0);
[data.PointableIds, info.PointableIds] = ros.internal.ros.messages.ros.default_type('int32',NaN);
info.MessageType = 'leap_motion/Gesture';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'lmc_gesture_id';
info.MatPath{2} = 'is_valid';
info.MatPath{3} = 'duration_us';
info.MatPath{4} = 'duration_s';
info.MatPath{5} = 'gesture_state';
info.MatPath{6} = 'gesture_type';
info.MatPath{7} = 'to_string';
info.MatPath{8} = 'pointable_ids';
