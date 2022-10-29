function [data, info] = ptuGotoFeedback
%PtuGotoFeedback gives an empty data for ptu_control/PtuGotoFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ptu_control/PtuGotoFeedback';
[data.State, info.State] = ros.internal.ros.messages.sensor_msgs.jointState;
info.State.MLdataType = 'struct';
info.MessageType = 'ptu_control/PtuGotoFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'state';
info.MatPath{2} = 'state.header';
info.MatPath{3} = 'state.header.seq';
info.MatPath{4} = 'state.header.stamp';
info.MatPath{5} = 'state.header.stamp.sec';
info.MatPath{6} = 'state.header.stamp.nsec';
info.MatPath{7} = 'state.header.frame_id';
info.MatPath{8} = 'state.name';
info.MatPath{9} = 'state.position';
info.MatPath{10} = 'state.velocity';
info.MatPath{11} = 'state.effort';
