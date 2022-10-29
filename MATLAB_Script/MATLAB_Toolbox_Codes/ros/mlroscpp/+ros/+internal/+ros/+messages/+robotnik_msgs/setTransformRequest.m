function [data, info] = setTransformRequest
%SetTransform gives an empty data for robotnik_msgs/SetTransformRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/SetTransformRequest';
[data.Tf, info.Tf] = ros.internal.ros.messages.geometry_msgs.transform;
info.Tf.MLdataType = 'struct';
info.MessageType = 'robotnik_msgs/SetTransformRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'tf';
info.MatPath{2} = 'tf.translation';
info.MatPath{3} = 'tf.translation.x';
info.MatPath{4} = 'tf.translation.y';
info.MatPath{5} = 'tf.translation.z';
info.MatPath{6} = 'tf.rotation';
info.MatPath{7} = 'tf.rotation.x';
info.MatPath{8} = 'tf.rotation.y';
info.MatPath{9} = 'tf.rotation.z';
info.MatPath{10} = 'tf.rotation.w';
