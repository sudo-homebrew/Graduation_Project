function [data, info] = bumperEvent
%BumperEvent gives an empty data for kobuki_msgs/BumperEvent

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'kobuki_msgs/BumperEvent';
[data.LEFT, info.LEFT] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.CENTER, info.CENTER] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.RIGHT, info.RIGHT] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.RELEASED, info.RELEASED] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.PRESSED, info.PRESSED] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.Bumper, info.Bumper] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.State, info.State] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'kobuki_msgs/BumperEvent';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'LEFT';
info.MatPath{2} = 'CENTER';
info.MatPath{3} = 'RIGHT';
info.MatPath{4} = 'RELEASED';
info.MatPath{5} = 'PRESSED';
info.MatPath{6} = 'bumper';
info.MatPath{7} = 'state';
