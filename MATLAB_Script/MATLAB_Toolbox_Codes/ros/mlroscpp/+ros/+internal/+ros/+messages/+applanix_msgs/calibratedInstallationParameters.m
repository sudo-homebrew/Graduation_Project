function [data, info] = calibratedInstallationParameters
%CalibratedInstallationParameters gives an empty data for applanix_msgs/CalibratedInstallationParameters

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/CalibratedInstallationParameters';
[data.Td, info.Td] = ros.internal.ros.messages.applanix_msgs.timeDistance;
info.Td.MLdataType = 'struct';
[data.STATUSPRIMARYGNSSLEVERARMCALIBRATING, info.STATUSPRIMARYGNSSLEVERARMCALIBRATING] = ros.internal.ros.messages.ros.default_type('uint16',1, 1);
[data.STATUSAUX1GNSSLEVERARMCALIBRATING, info.STATUSAUX1GNSSLEVERARMCALIBRATING] = ros.internal.ros.messages.ros.default_type('uint16',1, 2);
[data.STATUSAUX2GNSSLEVERARMCALIBRATING, info.STATUSAUX2GNSSLEVERARMCALIBRATING] = ros.internal.ros.messages.ros.default_type('uint16',1, 4);
[data.STATUSDMILEVERARMCALIBRATING, info.STATUSDMILEVERARMCALIBRATING] = ros.internal.ros.messages.ros.default_type('uint16',1, 8);
[data.STATUSDMISCALEFACTORCALIBRATING, info.STATUSDMISCALEFACTORCALIBRATING] = ros.internal.ros.messages.ros.default_type('uint16',1, 16);
[data.STATUSPOSITIONFIXLEVERARMCALIBRATING, info.STATUSPOSITIONFIXLEVERARMCALIBRATING] = ros.internal.ros.messages.ros.default_type('uint16',1, 64);
[data.STATUSPRIMARYGNSSLEVERARMCALIBRATED, info.STATUSPRIMARYGNSSLEVERARMCALIBRATED] = ros.internal.ros.messages.ros.default_type('uint16',1, 256);
[data.STATUSAUX1GNSSLEVERARMCALIBRATED, info.STATUSAUX1GNSSLEVERARMCALIBRATED] = ros.internal.ros.messages.ros.default_type('uint16',1, 512);
[data.STATUSAUX2GNSSLEVERARMCALIBRATED, info.STATUSAUX2GNSSLEVERARMCALIBRATED] = ros.internal.ros.messages.ros.default_type('uint16',1, 1024);
[data.STATUSDMILEVERARMCALIBRATED, info.STATUSDMILEVERARMCALIBRATED] = ros.internal.ros.messages.ros.default_type('uint16',1, 2048);
[data.STATUSDMISCALEFACTORCALIBRATED, info.STATUSDMISCALEFACTORCALIBRATED] = ros.internal.ros.messages.ros.default_type('uint16',1, 4096);
[data.STATUSPOSITIONFIXLEVERARMCALIBRATED, info.STATUSPOSITIONFIXLEVERARMCALIBRATED] = ros.internal.ros.messages.ros.default_type('uint16',1, 16384);
[data.Status, info.Status] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.PrimaryGnssLeverArm, info.PrimaryGnssLeverArm] = ros.internal.ros.messages.geometry_msgs.point32;
info.PrimaryGnssLeverArm.MLdataType = 'struct';
[data.PrimaryGnssLeverFom, info.PrimaryGnssLeverFom] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.Aux1GnssLeverArm, info.Aux1GnssLeverArm] = ros.internal.ros.messages.geometry_msgs.point32;
info.Aux1GnssLeverArm.MLdataType = 'struct';
[data.Aux1GnssLeverFom, info.Aux1GnssLeverFom] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.Aux2GnssLeverArm, info.Aux2GnssLeverArm] = ros.internal.ros.messages.geometry_msgs.point32;
info.Aux2GnssLeverArm.MLdataType = 'struct';
[data.Aux2GnssLeverFom, info.Aux2GnssLeverFom] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.DmiLeverArm, info.DmiLeverArm] = ros.internal.ros.messages.geometry_msgs.point32;
info.DmiLeverArm.MLdataType = 'struct';
[data.DmiLeverFom, info.DmiLeverFom] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.DmiScaleFactor, info.DmiScaleFactor] = ros.internal.ros.messages.ros.default_type('single',1);
[data.DmiScaleFactorFom, info.DmiScaleFactorFom] = ros.internal.ros.messages.ros.default_type('uint16',1);
info.MessageType = 'applanix_msgs/CalibratedInstallationParameters';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,41);
info.MatPath{1} = 'td';
info.MatPath{2} = 'td.time1';
info.MatPath{3} = 'td.time2';
info.MatPath{4} = 'td.distance';
info.MatPath{5} = 'td.time_types';
info.MatPath{6} = 'td.distance_type';
info.MatPath{7} = 'STATUS_PRIMARY_GNSS_LEVER_ARM_CALIBRATING';
info.MatPath{8} = 'STATUS_AUX_1_GNSS_LEVER_ARM_CALIBRATING';
info.MatPath{9} = 'STATUS_AUX_2_GNSS_LEVER_ARM_CALIBRATING';
info.MatPath{10} = 'STATUS_DMI_LEVER_ARM_CALIBRATING';
info.MatPath{11} = 'STATUS_DMI_SCALE_FACTOR_CALIBRATING';
info.MatPath{12} = 'STATUS_POSITION_FIX_LEVER_ARM_CALIBRATING';
info.MatPath{13} = 'STATUS_PRIMARY_GNSS_LEVER_ARM_CALIBRATED';
info.MatPath{14} = 'STATUS_AUX_1_GNSS_LEVER_ARM_CALIBRATED';
info.MatPath{15} = 'STATUS_AUX_2_GNSS_LEVER_ARM_CALIBRATED';
info.MatPath{16} = 'STATUS_DMI_LEVER_ARM_CALIBRATED';
info.MatPath{17} = 'STATUS_DMI_SCALE_FACTOR_CALIBRATED';
info.MatPath{18} = 'STATUS_POSITION_FIX_LEVER_ARM_CALIBRATED';
info.MatPath{19} = 'status';
info.MatPath{20} = 'primary_gnss_lever_arm';
info.MatPath{21} = 'primary_gnss_lever_arm.x';
info.MatPath{22} = 'primary_gnss_lever_arm.y';
info.MatPath{23} = 'primary_gnss_lever_arm.z';
info.MatPath{24} = 'primary_gnss_lever_fom';
info.MatPath{25} = 'aux_1_gnss_lever_arm';
info.MatPath{26} = 'aux_1_gnss_lever_arm.x';
info.MatPath{27} = 'aux_1_gnss_lever_arm.y';
info.MatPath{28} = 'aux_1_gnss_lever_arm.z';
info.MatPath{29} = 'aux_1_gnss_lever_fom';
info.MatPath{30} = 'aux_2_gnss_lever_arm';
info.MatPath{31} = 'aux_2_gnss_lever_arm.x';
info.MatPath{32} = 'aux_2_gnss_lever_arm.y';
info.MatPath{33} = 'aux_2_gnss_lever_arm.z';
info.MatPath{34} = 'aux_2_gnss_lever_fom';
info.MatPath{35} = 'dmi_lever_arm';
info.MatPath{36} = 'dmi_lever_arm.x';
info.MatPath{37} = 'dmi_lever_arm.y';
info.MatPath{38} = 'dmi_lever_arm.z';
info.MatPath{39} = 'dmi_lever_fom';
info.MatPath{40} = 'dmi_scale_factor';
info.MatPath{41} = 'dmi_scale_factor_fom';