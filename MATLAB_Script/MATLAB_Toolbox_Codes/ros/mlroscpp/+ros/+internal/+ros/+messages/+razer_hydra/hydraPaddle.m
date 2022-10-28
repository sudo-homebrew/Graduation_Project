function [data, info] = hydraPaddle
%HydraPaddle gives an empty data for razer_hydra/HydraPaddle

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'razer_hydra/HydraPaddle';
[data.Transform, info.Transform] = ros.internal.ros.messages.geometry_msgs.transform;
info.Transform.MLdataType = 'struct';
[data.Buttons, info.Buttons] = ros.internal.ros.messages.ros.default_type('logical',7);
[data.Joy, info.Joy] = ros.internal.ros.messages.ros.default_type('single',2);
[data.Trigger, info.Trigger] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'razer_hydra/HydraPaddle';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,13);
info.MatPath{1} = 'transform';
info.MatPath{2} = 'transform.translation';
info.MatPath{3} = 'transform.translation.x';
info.MatPath{4} = 'transform.translation.y';
info.MatPath{5} = 'transform.translation.z';
info.MatPath{6} = 'transform.rotation';
info.MatPath{7} = 'transform.rotation.x';
info.MatPath{8} = 'transform.rotation.y';
info.MatPath{9} = 'transform.rotation.z';
info.MatPath{10} = 'transform.rotation.w';
info.MatPath{11} = 'buttons';
info.MatPath{12} = 'joy';
info.MatPath{13} = 'trigger';
