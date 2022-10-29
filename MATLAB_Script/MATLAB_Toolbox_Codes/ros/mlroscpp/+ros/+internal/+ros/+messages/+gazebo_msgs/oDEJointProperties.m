function [data, info] = oDEJointProperties
%ODEJointProperties gives an empty data for gazebo_msgs/ODEJointProperties

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gazebo_msgs/ODEJointProperties';
[data.Damping, info.Damping] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.HiStop, info.HiStop] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.LoStop, info.LoStop] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.Erp, info.Erp] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.Cfm, info.Cfm] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.StopErp, info.StopErp] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.StopCfm, info.StopCfm] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.FudgeFactor, info.FudgeFactor] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.Fmax, info.Fmax] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.Vel, info.Vel] = ros.internal.ros.messages.ros.default_type('double',NaN);
info.MessageType = 'gazebo_msgs/ODEJointProperties';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'damping';
info.MatPath{2} = 'hiStop';
info.MatPath{3} = 'loStop';
info.MatPath{4} = 'erp';
info.MatPath{5} = 'cfm';
info.MatPath{6} = 'stop_erp';
info.MatPath{7} = 'stop_cfm';
info.MatPath{8} = 'fudge_factor';
info.MatPath{9} = 'fmax';
info.MatPath{10} = 'vel';
