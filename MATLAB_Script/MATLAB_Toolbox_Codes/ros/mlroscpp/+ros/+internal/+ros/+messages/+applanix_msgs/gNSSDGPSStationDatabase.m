function [data, info] = gNSSDGPSStationDatabase
%GNSSDGPSStationDatabase gives an empty data for applanix_msgs/GNSSDGPSStationDatabase

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/GNSSDGPSStationDatabase';
[data.Td, info.Td] = ros.internal.ros.messages.applanix_msgs.timeDistance;
info.Td.MLdataType = 'struct';
[data.Stations, info.Stations] = ros.internal.ros.messages.applanix_msgs.gNSSDGPSStation;
info.Stations.MLdataType = 'struct';
info.Stations.MaxLen = NaN;
info.Stations.MinLen = 0;
data.Stations = data.Stations([],1);
info.MessageType = 'applanix_msgs/GNSSDGPSStationDatabase';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,32);
info.MatPath{1} = 'td';
info.MatPath{2} = 'td.time1';
info.MatPath{3} = 'td.time2';
info.MatPath{4} = 'td.distance';
info.MatPath{5} = 'td.time_types';
info.MatPath{6} = 'td.distance_type';
info.MatPath{7} = 'stations';
info.MatPath{8} = 'stations.FLAGS_SATELLITE';
info.MatPath{9} = 'stations.FLAGS_STATION_PROVIDING_CORRECTIONS';
info.MatPath{10} = 'stations.FLAGS_STATION_USED_AS_RTCM_SOURCE';
info.MatPath{11} = 'stations.FLAGS_OMNISTAR_STATIONS';
info.MatPath{12} = 'stations.flags';
info.MatPath{13} = 'stations.id';
info.MatPath{14} = 'stations.frequency';
info.MatPath{15} = 'stations.HEALTH_NORMAL';
info.MatPath{16} = 'stations.HEALTH_NOT_MONITORED';
info.MatPath{17} = 'stations.HEALTH_NO_INFO_AVAILABLE';
info.MatPath{18} = 'stations.HEALTH_DO_NOT_USE';
info.MatPath{19} = 'stations.health';
info.MatPath{20} = 'stations.distance';
info.MatPath{21} = 'stations.range';
info.MatPath{22} = 'stations.uscg_index';
info.MatPath{23} = 'stations.seconds';
info.MatPath{24} = 'stations.RATE_25BPS';
info.MatPath{25} = 'stations.RATE_50BPS';
info.MatPath{26} = 'stations.RATE_100BPS';
info.MatPath{27} = 'stations.RATE_200BPS';
info.MatPath{28} = 'stations.RATE_600BPS';
info.MatPath{29} = 'stations.RATE_1200BPS';
info.MatPath{30} = 'stations.RATE_2400BPS';
info.MatPath{31} = 'stations.RATE_4800BPS';
info.MatPath{32} = 'stations.modulation_rate';