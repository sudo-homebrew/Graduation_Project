function [data, info] = headPanCommand
%HeadPanCommand gives an empty data for baxter_core_msgs/HeadPanCommand

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'baxter_core_msgs/HeadPanCommand';
[data.Target, info.Target] = ros.internal.ros.messages.ros.default_type('single',1);
[data.SpeedRatio, info.SpeedRatio] = ros.internal.ros.messages.ros.default_type('single',1);
[data.MAXSPEEDRATIO, info.MAXSPEEDRATIO] = ros.internal.ros.messages.ros.default_type('single',1, 1);
[data.MINSPEEDRATIO, info.MINSPEEDRATIO] = ros.internal.ros.messages.ros.default_type('single',1, 0);
[data.EnablePanRequest, info.EnablePanRequest] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.REQUESTPANDISABLE, info.REQUESTPANDISABLE] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.REQUESTPANENABLE, info.REQUESTPANENABLE] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.REQUESTPANVOID, info.REQUESTPANVOID] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
info.MessageType = 'baxter_core_msgs/HeadPanCommand';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'target';
info.MatPath{2} = 'speed_ratio';
info.MatPath{3} = 'MAX_SPEED_RATIO';
info.MatPath{4} = 'MIN_SPEED_RATIO';
info.MatPath{5} = 'enable_pan_request';
info.MatPath{6} = 'REQUEST_PAN_DISABLE';
info.MatPath{7} = 'REQUEST_PAN_ENABLE';
info.MatPath{8} = 'REQUEST_PAN_VOID';
