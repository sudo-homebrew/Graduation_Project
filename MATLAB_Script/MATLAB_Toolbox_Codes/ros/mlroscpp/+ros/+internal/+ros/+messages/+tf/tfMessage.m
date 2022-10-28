function [data, info] = tfMessage
%tfMessage gives an empty data for tf/tfMessage

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'tf/tfMessage';
[data.Transforms, info.Transforms] = ros.internal.ros.messages.geometry_msgs.transformStamped;
info.Transforms.MLdataType = 'struct';
info.Transforms.MaxLen = NaN;
info.Transforms.MinLen = 0;
data.Transforms = data.Transforms([],1);
info.MessageType = 'tf/tfMessage';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,18);
info.MatPath{1} = 'transforms';
info.MatPath{2} = 'transforms.header';
info.MatPath{3} = 'transforms.header.seq';
info.MatPath{4} = 'transforms.header.stamp';
info.MatPath{5} = 'transforms.header.stamp.sec';
info.MatPath{6} = 'transforms.header.stamp.nsec';
info.MatPath{7} = 'transforms.header.frame_id';
info.MatPath{8} = 'transforms.child_frame_id';
info.MatPath{9} = 'transforms.transform';
info.MatPath{10} = 'transforms.transform.translation';
info.MatPath{11} = 'transforms.transform.translation.x';
info.MatPath{12} = 'transforms.transform.translation.y';
info.MatPath{13} = 'transforms.transform.translation.z';
info.MatPath{14} = 'transforms.transform.rotation';
info.MatPath{15} = 'transforms.transform.rotation.x';
info.MatPath{16} = 'transforms.transform.rotation.y';
info.MatPath{17} = 'transforms.transform.rotation.z';
info.MatPath{18} = 'transforms.transform.rotation.w';
