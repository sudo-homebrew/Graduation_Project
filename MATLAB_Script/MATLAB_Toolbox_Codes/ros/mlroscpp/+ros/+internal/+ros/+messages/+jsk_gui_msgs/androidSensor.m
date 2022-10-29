function [data, info] = androidSensor
%AndroidSensor gives an empty data for jsk_gui_msgs/AndroidSensor

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_gui_msgs/AndroidSensor';
[data.AccelX, info.AccelX] = ros.internal.ros.messages.ros.default_type('double',1);
[data.AccelY, info.AccelY] = ros.internal.ros.messages.ros.default_type('double',1);
[data.AccelZ, info.AccelZ] = ros.internal.ros.messages.ros.default_type('double',1);
[data.OrientationX, info.OrientationX] = ros.internal.ros.messages.ros.default_type('double',1);
[data.OrientationY, info.OrientationY] = ros.internal.ros.messages.ros.default_type('double',1);
[data.OrientationZ, info.OrientationZ] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'jsk_gui_msgs/AndroidSensor';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'accel_x';
info.MatPath{2} = 'accel_y';
info.MatPath{3} = 'accel_z';
info.MatPath{4} = 'orientation_x';
info.MatPath{5} = 'orientation_y';
info.MatPath{6} = 'orientation_z';
