function [data, info] = ambience
%Ambience gives an empty data for grizzly_msgs/Ambience

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'grizzly_msgs/Ambience';
[data.LIGHTSFRONTLEFT, info.LIGHTSFRONTLEFT] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.LIGHTSFRONTRIGHT, info.LIGHTSFRONTRIGHT] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.LIGHTSREARLEFT, info.LIGHTSREARLEFT] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.LIGHTSREARRIGHT, info.LIGHTSREARRIGHT] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.BodyLights, info.BodyLights] = ros.internal.ros.messages.ros.default_type('uint8',4);
[data.Beacon, info.Beacon] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Beep, info.Beep] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'grizzly_msgs/Ambience';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'LIGHTS_FRONT_LEFT';
info.MatPath{2} = 'LIGHTS_FRONT_RIGHT';
info.MatPath{3} = 'LIGHTS_REAR_LEFT';
info.MatPath{4} = 'LIGHTS_REAR_RIGHT';
info.MatPath{5} = 'body_lights';
info.MatPath{6} = 'beacon';
info.MatPath{7} = 'beep';
