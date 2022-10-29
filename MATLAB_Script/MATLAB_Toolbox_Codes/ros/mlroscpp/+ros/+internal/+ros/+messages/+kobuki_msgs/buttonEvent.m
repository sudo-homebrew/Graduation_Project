function [data, info] = buttonEvent
%ButtonEvent gives an empty data for kobuki_msgs/ButtonEvent

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'kobuki_msgs/ButtonEvent';
[data.BUTTON0, info.BUTTON0] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.BUTTON1, info.BUTTON1] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.BUTTON2, info.BUTTON2] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.RELEASED, info.RELEASED] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.PRESSED, info.PRESSED] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.Button, info.Button] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.State, info.State] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'kobuki_msgs/ButtonEvent';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'BUTTON0';
info.MatPath{2} = 'BUTTON1';
info.MatPath{3} = 'BUTTON2';
info.MatPath{4} = 'RELEASED';
info.MatPath{5} = 'PRESSED';
info.MatPath{6} = 'button';
info.MatPath{7} = 'state';
