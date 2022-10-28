function [data, info] = magneticField
%MagneticField gives an empty data for jsk_gui_msgs/MagneticField

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_gui_msgs/MagneticField';
[data.Magneticfield, info.Magneticfield] = ros.internal.ros.messages.geometry_msgs.vector3;
info.Magneticfield.MLdataType = 'struct';
info.MessageType = 'jsk_gui_msgs/MagneticField';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'magneticfield';
info.MatPath{2} = 'magneticfield.x';
info.MatPath{3} = 'magneticfield.y';
info.MatPath{4} = 'magneticfield.z';
