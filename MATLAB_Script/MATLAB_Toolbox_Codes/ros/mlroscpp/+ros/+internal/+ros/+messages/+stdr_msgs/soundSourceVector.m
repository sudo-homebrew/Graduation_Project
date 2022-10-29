function [data, info] = soundSourceVector
%SoundSourceVector gives an empty data for stdr_msgs/SoundSourceVector

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'stdr_msgs/SoundSourceVector';
[data.SoundSources, info.SoundSources] = ros.internal.ros.messages.stdr_msgs.soundSource;
info.SoundSources.MLdataType = 'struct';
info.SoundSources.MaxLen = NaN;
info.SoundSources.MinLen = 0;
data.SoundSources = data.SoundSources([],1);
info.MessageType = 'stdr_msgs/SoundSourceVector';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'sound_sources';
info.MatPath{2} = 'sound_sources.id';
info.MatPath{3} = 'sound_sources.dbs';
info.MatPath{4} = 'sound_sources.pose';
info.MatPath{5} = 'sound_sources.pose.x';
info.MatPath{6} = 'sound_sources.pose.y';
info.MatPath{7} = 'sound_sources.pose.theta';
