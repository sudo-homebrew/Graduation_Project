function [data, info] = wheelDropEvent
%WheelDropEvent gives an empty data for kobuki_msgs/WheelDropEvent

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'kobuki_msgs/WheelDropEvent';
[data.LEFT, info.LEFT] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.RIGHT, info.RIGHT] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.RAISED, info.RAISED] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.DROPPED, info.DROPPED] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.Wheel, info.Wheel] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.State, info.State] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'kobuki_msgs/WheelDropEvent';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'LEFT';
info.MatPath{2} = 'RIGHT';
info.MatPath{3} = 'RAISED';
info.MatPath{4} = 'DROPPED';
info.MatPath{5} = 'wheel';
info.MatPath{6} = 'state';
