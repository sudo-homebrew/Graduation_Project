function [data, info] = laserTrajCmd
%LaserTrajCmd gives an empty data for pr2_msgs/LaserTrajCmd

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_msgs/LaserTrajCmd';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Profile, info.Profile] = ros.internal.ros.messages.ros.char('string',0);
[data.Position, info.Position] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.TimeFromStart, info.TimeFromStart] = ros.internal.ros.messages.ros.duration;
info.TimeFromStart.MLdataType = 'struct';
info.TimeFromStart.MaxLen = NaN;
info.TimeFromStart.MinLen = 0;
data.TimeFromStart = data.TimeFromStart([],1);
[data.MaxVelocity, info.MaxVelocity] = ros.internal.ros.messages.ros.default_type('double',1);
[data.MaxAcceleration, info.MaxAcceleration] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'pr2_msgs/LaserTrajCmd';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,13);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'profile';
info.MatPath{8} = 'position';
info.MatPath{9} = 'time_from_start';
info.MatPath{10} = 'time_from_start.sec';
info.MatPath{11} = 'time_from_start.nsec';
info.MatPath{12} = 'max_velocity';
info.MatPath{13} = 'max_acceleration';
