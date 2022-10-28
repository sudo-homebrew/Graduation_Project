function [data, info] = dGPSSourceControl
%DGPSSourceControl gives an empty data for applanix_msgs/DGPSSourceControl

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/DGPSSourceControl';
[data.Transaction, info.Transaction] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.DgpsSourceMode, info.DgpsSourceMode] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.BeaconAcquisitionMode, info.BeaconAcquisitionMode] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.BeaconChannel0Frequency, info.BeaconChannel0Frequency] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.BeaconChannel1Frequency, info.BeaconChannel1Frequency] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.SatelliteId, info.SatelliteId] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.SatelliteBitrate, info.SatelliteBitrate] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.SatelliteFrequency, info.SatelliteFrequency] = ros.internal.ros.messages.ros.default_type('double',1);
[data.RequestDatabaseSource, info.RequestDatabaseSource] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.LandstarCorrectionSource, info.LandstarCorrectionSource] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.OmnistarActivationCode, info.OmnistarActivationCode] = ros.internal.ros.messages.ros.default_type('uint8',26);
info.MessageType = 'applanix_msgs/DGPSSourceControl';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'transaction';
info.MatPath{2} = 'dgps_source_mode';
info.MatPath{3} = 'beacon_acquisition_mode';
info.MatPath{4} = 'beacon_channel_0_frequency';
info.MatPath{5} = 'beacon_channel_1_frequency';
info.MatPath{6} = 'satellite_id';
info.MatPath{7} = 'satellite_bitrate';
info.MatPath{8} = 'satellite_frequency';
info.MatPath{9} = 'request_database_source';
info.MatPath{10} = 'landstar_correction_source';
info.MatPath{11} = 'omnistar_activation_code';
