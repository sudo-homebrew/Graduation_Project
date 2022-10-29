function [data, info] = gravity
%Gravity gives an empty data for jsk_gui_msgs/Gravity

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_gui_msgs/Gravity';
[data.Gravity_, info.Gravity_] = ros.internal.ros.messages.geometry_msgs.vector3;
info.Gravity_.MLdataType = 'struct';
info.MessageType = 'jsk_gui_msgs/Gravity';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'gravity';
info.MatPath{2} = 'gravity.x';
info.MatPath{3} = 'gravity.y';
info.MatPath{4} = 'gravity.z';
