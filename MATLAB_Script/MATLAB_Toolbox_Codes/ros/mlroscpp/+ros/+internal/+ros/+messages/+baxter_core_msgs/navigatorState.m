function [data, info] = navigatorState
%NavigatorState gives an empty data for baxter_core_msgs/NavigatorState

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'baxter_core_msgs/NavigatorState';
[data.ButtonNames, info.ButtonNames] = ros.internal.ros.messages.ros.char('string',NaN);
[data.Buttons, info.Buttons] = ros.internal.ros.messages.ros.default_type('logical',NaN);
[data.Wheel, info.Wheel] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.LightNames, info.LightNames] = ros.internal.ros.messages.ros.char('string',NaN);
[data.Lights, info.Lights] = ros.internal.ros.messages.ros.default_type('logical',NaN);
info.MessageType = 'baxter_core_msgs/NavigatorState';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'button_names';
info.MatPath{2} = 'buttons';
info.MatPath{3} = 'wheel';
info.MatPath{4} = 'light_names';
info.MatPath{5} = 'lights';
