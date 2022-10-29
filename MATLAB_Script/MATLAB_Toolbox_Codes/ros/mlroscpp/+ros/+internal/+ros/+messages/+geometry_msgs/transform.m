function [data, info] = transform
%Transform gives an empty data for geometry_msgs/Transform

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/Transform';
[data.Translation, info.Translation] = ros.internal.ros.messages.geometry_msgs.vector3;
info.Translation.MLdataType = 'struct';
[data.Rotation, info.Rotation] = ros.internal.ros.messages.geometry_msgs.quaternion;
info.Rotation.MLdataType = 'struct';
info.MessageType = 'geometry_msgs/Transform';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'translation';
info.MatPath{2} = 'translation.x';
info.MatPath{3} = 'translation.y';
info.MatPath{4} = 'translation.z';
info.MatPath{5} = 'rotation';
info.MatPath{6} = 'rotation.x';
info.MatPath{7} = 'rotation.y';
info.MatPath{8} = 'rotation.z';
info.MatPath{9} = 'rotation.w';
