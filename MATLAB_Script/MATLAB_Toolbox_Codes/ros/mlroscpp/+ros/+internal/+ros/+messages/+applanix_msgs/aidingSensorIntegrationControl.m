function [data, info] = aidingSensorIntegrationControl
%AidingSensorIntegrationControl gives an empty data for applanix_msgs/AidingSensorIntegrationControl

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/AidingSensorIntegrationControl';
[data.Transaction, info.Transaction] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.OVERRIDEFORCEPRIMARYGNSSVALID, info.OVERRIDEFORCEPRIMARYGNSSVALID] = ros.internal.ros.messages.ros.default_type('uint32',1, 1);
[data.OVERRIDEFORCEPRIMARYGNSSINVALID, info.OVERRIDEFORCEPRIMARYGNSSINVALID] = ros.internal.ros.messages.ros.default_type('uint32',1, 2);
[data.OVERRIDEFORCEAUXGNSSVALID, info.OVERRIDEFORCEAUXGNSSVALID] = ros.internal.ros.messages.ros.default_type('uint32',1, 4);
[data.OVERRIDEFORCEAUXGNSSINVALID, info.OVERRIDEFORCEAUXGNSSINVALID] = ros.internal.ros.messages.ros.default_type('uint32',1, 8);
[data.OVERRIDEDISABLEGAMSHEADINGAIDING, info.OVERRIDEDISABLEGAMSHEADINGAIDING] = ros.internal.ros.messages.ros.default_type('uint32',1, 16);
[data.OVERRIDEFORCEDMIVALID, info.OVERRIDEFORCEDMIVALID] = ros.internal.ros.messages.ros.default_type('uint32',1, 32);
[data.OVERRIDEFORCEDMIINVALID, info.OVERRIDEFORCEDMIINVALID] = ros.internal.ros.messages.ros.default_type('uint32',1, 64);
[data.OVERRIDEDISABLEYZVELOCITYAIDING, info.OVERRIDEDISABLEYZVELOCITYAIDING] = ros.internal.ros.messages.ros.default_type('uint32',1, 128);
[data.Override, info.Override] = ros.internal.ros.messages.ros.default_type('uint32',1);
info.MessageType = 'applanix_msgs/AidingSensorIntegrationControl';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'transaction';
info.MatPath{2} = 'OVERRIDE_FORCE_PRIMARY_GNSS_VALID';
info.MatPath{3} = 'OVERRIDE_FORCE_PRIMARY_GNSS_INVALID';
info.MatPath{4} = 'OVERRIDE_FORCE_AUX_GNSS_VALID';
info.MatPath{5} = 'OVERRIDE_FORCE_AUX_GNSS_INVALID';
info.MatPath{6} = 'OVERRIDE_DISABLE_GAMS_HEADING_AIDING';
info.MatPath{7} = 'OVERRIDE_FORCE_DMI_VALID';
info.MatPath{8} = 'OVERRIDE_FORCE_DMI_INVALID';
info.MatPath{9} = 'OVERRIDE_DISABLE_YZ_VELOCITY_AIDING';
info.MatPath{10} = 'override';
