function [data, info] = gNSSDGPSStation
%GNSSDGPSStation gives an empty data for applanix_msgs/GNSSDGPSStation

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/GNSSDGPSStation';
[data.FLAGSSATELLITE, info.FLAGSSATELLITE] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.FLAGSSTATIONPROVIDINGCORRECTIONS, info.FLAGSSTATIONPROVIDINGCORRECTIONS] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.FLAGSSTATIONUSEDASRTCMSOURCE, info.FLAGSSTATIONUSEDASRTCMSOURCE] = ros.internal.ros.messages.ros.default_type('uint8',1, 4);
[data.FLAGSOMNISTARSTATIONS, info.FLAGSOMNISTARSTATIONS] = ros.internal.ros.messages.ros.default_type('uint8',1, 8);
[data.Flags, info.Flags] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Id, info.Id] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.Frequency, info.Frequency] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.HEALTHNORMAL, info.HEALTHNORMAL] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.HEALTHNOTMONITORED, info.HEALTHNOTMONITORED] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.HEALTHNOINFOAVAILABLE, info.HEALTHNOINFOAVAILABLE] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.HEALTHDONOTUSE, info.HEALTHDONOTUSE] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.Health, info.Health] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Distance, info.Distance] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Range, info.Range] = ros.internal.ros.messages.ros.default_type('single',1);
[data.UscgIndex, info.UscgIndex] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Seconds, info.Seconds] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.RATE25BPS, info.RATE25BPS] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.RATE50BPS, info.RATE50BPS] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.RATE100BPS, info.RATE100BPS] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.RATE200BPS, info.RATE200BPS] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.RATE600BPS, info.RATE600BPS] = ros.internal.ros.messages.ros.default_type('uint8',1, 4);
[data.RATE1200BPS, info.RATE1200BPS] = ros.internal.ros.messages.ros.default_type('uint8',1, 5);
[data.RATE2400BPS, info.RATE2400BPS] = ros.internal.ros.messages.ros.default_type('uint8',1, 6);
[data.RATE4800BPS, info.RATE4800BPS] = ros.internal.ros.messages.ros.default_type('uint8',1, 7);
[data.ModulationRate, info.ModulationRate] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'applanix_msgs/GNSSDGPSStation';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,25);
info.MatPath{1} = 'FLAGS_SATELLITE';
info.MatPath{2} = 'FLAGS_STATION_PROVIDING_CORRECTIONS';
info.MatPath{3} = 'FLAGS_STATION_USED_AS_RTCM_SOURCE';
info.MatPath{4} = 'FLAGS_OMNISTAR_STATIONS';
info.MatPath{5} = 'flags';
info.MatPath{6} = 'id';
info.MatPath{7} = 'frequency';
info.MatPath{8} = 'HEALTH_NORMAL';
info.MatPath{9} = 'HEALTH_NOT_MONITORED';
info.MatPath{10} = 'HEALTH_NO_INFO_AVAILABLE';
info.MatPath{11} = 'HEALTH_DO_NOT_USE';
info.MatPath{12} = 'health';
info.MatPath{13} = 'distance';
info.MatPath{14} = 'range';
info.MatPath{15} = 'uscg_index';
info.MatPath{16} = 'seconds';
info.MatPath{17} = 'RATE_25BPS';
info.MatPath{18} = 'RATE_50BPS';
info.MatPath{19} = 'RATE_100BPS';
info.MatPath{20} = 'RATE_200BPS';
info.MatPath{21} = 'RATE_600BPS';
info.MatPath{22} = 'RATE_1200BPS';
info.MatPath{23} = 'RATE_2400BPS';
info.MatPath{24} = 'RATE_4800BPS';
info.MatPath{25} = 'modulation_rate';