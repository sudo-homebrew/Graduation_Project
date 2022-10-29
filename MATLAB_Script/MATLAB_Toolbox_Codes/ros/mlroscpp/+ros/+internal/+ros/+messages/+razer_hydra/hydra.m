function [data, info] = hydra
%Hydra gives an empty data for razer_hydra/Hydra

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'razer_hydra/Hydra';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Paddles, info.Paddles] = ros.internal.ros.messages.razer_hydra.hydraPaddle;
info.Paddles.MLdataType = 'struct';
info.Paddles.MaxLen = 2;
info.Paddles.MinLen = 2;
[data.LEFT, info.LEFT] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.RIGHT, info.RIGHT] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
val = [];
for i = 1:2
    val = vertcat(data.Paddles, val); %#ok<AGROW>
end
data.Paddles = val;
info.MessageType = 'razer_hydra/Hydra';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,22);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'paddles';
info.MatPath{8} = 'paddles.transform';
info.MatPath{9} = 'paddles.transform.translation';
info.MatPath{10} = 'paddles.transform.translation.x';
info.MatPath{11} = 'paddles.transform.translation.y';
info.MatPath{12} = 'paddles.transform.translation.z';
info.MatPath{13} = 'paddles.transform.rotation';
info.MatPath{14} = 'paddles.transform.rotation.x';
info.MatPath{15} = 'paddles.transform.rotation.y';
info.MatPath{16} = 'paddles.transform.rotation.z';
info.MatPath{17} = 'paddles.transform.rotation.w';
info.MatPath{18} = 'paddles.buttons';
info.MatPath{19} = 'paddles.joy';
info.MatPath{20} = 'paddles.trigger';
info.MatPath{21} = 'LEFT';
info.MatPath{22} = 'RIGHT';
