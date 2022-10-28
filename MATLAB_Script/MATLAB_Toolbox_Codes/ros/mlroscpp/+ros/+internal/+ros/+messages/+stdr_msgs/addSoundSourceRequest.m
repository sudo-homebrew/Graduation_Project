function [data, info] = addSoundSourceRequest
%AddSoundSource gives an empty data for stdr_msgs/AddSoundSourceRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'stdr_msgs/AddSoundSourceRequest';
[data.NewSource, info.NewSource] = ros.internal.ros.messages.stdr_msgs.soundSource;
info.NewSource.MLdataType = 'struct';
info.MessageType = 'stdr_msgs/AddSoundSourceRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'newSource';
info.MatPath{2} = 'newSource.id';
info.MatPath{3} = 'newSource.dbs';
info.MatPath{4} = 'newSource.pose';
info.MatPath{5} = 'newSource.pose.x';
info.MatPath{6} = 'newSource.pose.y';
info.MatPath{7} = 'newSource.pose.theta';
