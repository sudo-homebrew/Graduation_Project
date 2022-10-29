function [data, info] = forceControllerRequest
%ForceController gives an empty data for sr_robot_msgs/ForceControllerRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/ForceControllerRequest';
[data.Maxpwm, info.Maxpwm] = ros.internal.ros.messages.ros.default_type('int16',1);
[data.Sgleftref, info.Sgleftref] = ros.internal.ros.messages.ros.default_type('int16',1);
[data.Sgrightref, info.Sgrightref] = ros.internal.ros.messages.ros.default_type('int16',1);
[data.F, info.F] = ros.internal.ros.messages.ros.default_type('int16',1);
[data.P, info.P] = ros.internal.ros.messages.ros.default_type('int16',1);
[data.I, info.I] = ros.internal.ros.messages.ros.default_type('int16',1);
[data.D, info.D] = ros.internal.ros.messages.ros.default_type('int16',1);
[data.Imax, info.Imax] = ros.internal.ros.messages.ros.default_type('int16',1);
[data.Deadband, info.Deadband] = ros.internal.ros.messages.ros.default_type('int16',1);
[data.Sign, info.Sign] = ros.internal.ros.messages.ros.default_type('int16',1);
info.MessageType = 'sr_robot_msgs/ForceControllerRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'maxpwm';
info.MatPath{2} = 'sgleftref';
info.MatPath{3} = 'sgrightref';
info.MatPath{4} = 'f';
info.MatPath{5} = 'p';
info.MatPath{6} = 'i';
info.MatPath{7} = 'd';
info.MatPath{8} = 'imax';
info.MatPath{9} = 'deadband';
info.MatPath{10} = 'sign';
