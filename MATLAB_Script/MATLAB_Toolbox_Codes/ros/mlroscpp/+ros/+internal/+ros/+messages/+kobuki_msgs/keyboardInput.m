function [data, info] = keyboardInput
%KeyboardInput gives an empty data for kobuki_msgs/KeyboardInput

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'kobuki_msgs/KeyboardInput';
[data.KEYCODERIGHT, info.KEYCODERIGHT] = ros.internal.ros.messages.ros.default_type('uint8',1, 67);
[data.KEYCODELEFT, info.KEYCODELEFT] = ros.internal.ros.messages.ros.default_type('uint8',1, 68);
[data.KEYCODEUP, info.KEYCODEUP] = ros.internal.ros.messages.ros.default_type('uint8',1, 65);
[data.KEYCODEDOWN, info.KEYCODEDOWN] = ros.internal.ros.messages.ros.default_type('uint8',1, 66);
[data.KEYCODESPACE, info.KEYCODESPACE] = ros.internal.ros.messages.ros.default_type('uint8',1, 32);
[data.KEYCODEENABLE, info.KEYCODEENABLE] = ros.internal.ros.messages.ros.default_type('uint8',1, 101);
[data.KEYCODEDISABLE, info.KEYCODEDISABLE] = ros.internal.ros.messages.ros.default_type('uint8',1, 100);
[data.PressedKey, info.PressedKey] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'kobuki_msgs/KeyboardInput';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'KEYCODE_RIGHT';
info.MatPath{2} = 'KEYCODE_LEFT';
info.MatPath{3} = 'KEYCODE_UP';
info.MatPath{4} = 'KEYCODE_DOWN';
info.MatPath{5} = 'KEYCODE_SPACE';
info.MatPath{6} = 'KEYCODE_ENABLE';
info.MatPath{7} = 'KEYCODE_DISABLE';
info.MatPath{8} = 'pressed_key';
