function [data, info] = switchControllerRequest
%SwitchController gives an empty data for controller_manager_msgs/SwitchControllerRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'controller_manager_msgs/SwitchControllerRequest';
[data.StartControllers, info.StartControllers] = ros.internal.ros.messages.ros.char('string',NaN);
[data.StopControllers, info.StopControllers] = ros.internal.ros.messages.ros.char('string',NaN);
[data.Strictness, info.Strictness] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.BESTEFFORT, info.BESTEFFORT] = ros.internal.ros.messages.ros.default_type('int32',1, 1);
[data.STRICT, info.STRICT] = ros.internal.ros.messages.ros.default_type('int32',1, 2);
[data.StartAsap, info.StartAsap] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Timeout, info.Timeout] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'controller_manager_msgs/SwitchControllerRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'start_controllers';
info.MatPath{2} = 'stop_controllers';
info.MatPath{3} = 'strictness';
info.MatPath{4} = 'BEST_EFFORT';
info.MatPath{5} = 'STRICT';
info.MatPath{6} = 'start_asap';
info.MatPath{7} = 'timeout';
