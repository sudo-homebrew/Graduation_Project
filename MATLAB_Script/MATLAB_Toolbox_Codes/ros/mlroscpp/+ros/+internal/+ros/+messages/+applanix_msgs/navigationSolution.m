function [data, info] = navigationSolution
%NavigationSolution gives an empty data for applanix_msgs/NavigationSolution

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/NavigationSolution';
[data.Td, info.Td] = ros.internal.ros.messages.applanix_msgs.timeDistance;
info.Td.MLdataType = 'struct';
[data.Latitude, info.Latitude] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Longitude, info.Longitude] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Altitude, info.Altitude] = ros.internal.ros.messages.ros.default_type('double',1);
[data.NorthVel, info.NorthVel] = ros.internal.ros.messages.ros.default_type('single',1);
[data.EastVel, info.EastVel] = ros.internal.ros.messages.ros.default_type('single',1);
[data.DownVel, info.DownVel] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Roll, info.Roll] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Pitch, info.Pitch] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Heading, info.Heading] = ros.internal.ros.messages.ros.default_type('double',1);
[data.WanderAngle, info.WanderAngle] = ros.internal.ros.messages.ros.default_type('double',1);
[data.TrackAngle, info.TrackAngle] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Speed, info.Speed] = ros.internal.ros.messages.ros.default_type('single',1);
[data.AngRateLong, info.AngRateLong] = ros.internal.ros.messages.ros.default_type('single',1);
[data.AngRateTrans, info.AngRateTrans] = ros.internal.ros.messages.ros.default_type('single',1);
[data.AngRateDown, info.AngRateDown] = ros.internal.ros.messages.ros.default_type('single',1);
[data.LongAccel, info.LongAccel] = ros.internal.ros.messages.ros.default_type('single',1);
[data.TransAccel, info.TransAccel] = ros.internal.ros.messages.ros.default_type('single',1);
[data.DownAccel, info.DownAccel] = ros.internal.ros.messages.ros.default_type('single',1);
[data.ALIGNMENTFULLNAVIGATION, info.ALIGNMENTFULLNAVIGATION] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.ALIGNMENTFINEALIGNMENTACTIVE, info.ALIGNMENTFINEALIGNMENTACTIVE] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.ALIGNMENTGCCHI2, info.ALIGNMENTGCCHI2] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.ALIGNMENTPCCHI2, info.ALIGNMENTPCCHI2] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.ALIGNMENTGCCHI1, info.ALIGNMENTGCCHI1] = ros.internal.ros.messages.ros.default_type('uint8',1, 4);
[data.ALIGNMENTPCCHI1, info.ALIGNMENTPCCHI1] = ros.internal.ros.messages.ros.default_type('uint8',1, 5);
[data.ALIGNMENTCOARSELEVELING, info.ALIGNMENTCOARSELEVELING] = ros.internal.ros.messages.ros.default_type('uint8',1, 6);
[data.ALIGNMENTINITIALSOLUTION, info.ALIGNMENTINITIALSOLUTION] = ros.internal.ros.messages.ros.default_type('uint8',1, 7);
[data.ALIGNMENTNOVALIDSOLUTION, info.ALIGNMENTNOVALIDSOLUTION] = ros.internal.ros.messages.ros.default_type('uint8',1, 8);
[data.AlignmentStatus, info.AlignmentStatus] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'applanix_msgs/NavigationSolution';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,34);
info.MatPath{1} = 'td';
info.MatPath{2} = 'td.time1';
info.MatPath{3} = 'td.time2';
info.MatPath{4} = 'td.distance';
info.MatPath{5} = 'td.time_types';
info.MatPath{6} = 'td.distance_type';
info.MatPath{7} = 'latitude';
info.MatPath{8} = 'longitude';
info.MatPath{9} = 'altitude';
info.MatPath{10} = 'north_vel';
info.MatPath{11} = 'east_vel';
info.MatPath{12} = 'down_vel';
info.MatPath{13} = 'roll';
info.MatPath{14} = 'pitch';
info.MatPath{15} = 'heading';
info.MatPath{16} = 'wander_angle';
info.MatPath{17} = 'track_angle';
info.MatPath{18} = 'speed';
info.MatPath{19} = 'ang_rate_long';
info.MatPath{20} = 'ang_rate_trans';
info.MatPath{21} = 'ang_rate_down';
info.MatPath{22} = 'long_accel';
info.MatPath{23} = 'trans_accel';
info.MatPath{24} = 'down_accel';
info.MatPath{25} = 'ALIGNMENT_FULL_NAVIGATION';
info.MatPath{26} = 'ALIGNMENT_FINE_ALIGNMENT_ACTIVE';
info.MatPath{27} = 'ALIGNMENT_GC_CHI_2';
info.MatPath{28} = 'ALIGNMENT_PC_CHI_2';
info.MatPath{29} = 'ALIGNMENT_GC_CHI_1';
info.MatPath{30} = 'ALIGNMENT_PC_CHI_1';
info.MatPath{31} = 'ALIGNMENT_COARSE_LEVELING';
info.MatPath{32} = 'ALIGNMENT_INITIAL_SOLUTION';
info.MatPath{33} = 'ALIGNMENT_NO_VALID_SOLUTION';
info.MatPath{34} = 'alignment_status';