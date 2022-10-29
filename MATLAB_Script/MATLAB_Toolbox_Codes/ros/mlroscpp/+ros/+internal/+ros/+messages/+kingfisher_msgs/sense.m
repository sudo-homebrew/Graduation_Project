function [data, info] = sense
%Sense gives an empty data for kingfisher_msgs/Sense

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'kingfisher_msgs/Sense';
[data.Battery, info.Battery] = ros.internal.ros.messages.ros.default_type('single',1);
[data.CurrentLeft, info.CurrentLeft] = ros.internal.ros.messages.ros.default_type('single',1);
[data.CurrentRight, info.CurrentRight] = ros.internal.ros.messages.ros.default_type('single',1);
[data.PcbTemperature, info.PcbTemperature] = ros.internal.ros.messages.ros.default_type('single',1);
[data.FanOn, info.FanOn] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.RCINRANGE, info.RCINRANGE] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.RCINUSE, info.RCINUSE] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.Rc, info.Rc] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.RcThrottle, info.RcThrottle] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.RcRotation, info.RcRotation] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.RcEnable, info.RcEnable] = ros.internal.ros.messages.ros.default_type('uint16',1);
info.MessageType = 'kingfisher_msgs/Sense';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'battery';
info.MatPath{2} = 'current_left';
info.MatPath{3} = 'current_right';
info.MatPath{4} = 'pcb_temperature';
info.MatPath{5} = 'fan_on';
info.MatPath{6} = 'RC_INRANGE';
info.MatPath{7} = 'RC_INUSE';
info.MatPath{8} = 'rc';
info.MatPath{9} = 'rc_throttle';
info.MatPath{10} = 'rc_rotation';
info.MatPath{11} = 'rc_enable';
