function [data, info] = hydraRaw
%HydraRaw gives an empty data for razer_hydra/HydraRaw

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'razer_hydra/HydraRaw';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Pos, info.Pos] = ros.internal.ros.messages.ros.default_type('int16',6);
[data.Quat, info.Quat] = ros.internal.ros.messages.ros.default_type('int16',8);
[data.Buttons, info.Buttons] = ros.internal.ros.messages.ros.default_type('uint8',2);
[data.Analog, info.Analog] = ros.internal.ros.messages.ros.default_type('int16',6);
info.MessageType = 'razer_hydra/HydraRaw';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'pos';
info.MatPath{8} = 'quat';
info.MatPath{9} = 'buttons';
info.MatPath{10} = 'analog';
