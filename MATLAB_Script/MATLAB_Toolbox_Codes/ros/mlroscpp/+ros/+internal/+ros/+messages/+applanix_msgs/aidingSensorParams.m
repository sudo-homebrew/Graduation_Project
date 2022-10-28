function [data, info] = aidingSensorParams
%AidingSensorParams gives an empty data for applanix_msgs/AidingSensorParams

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/AidingSensorParams';
[data.Transaction, info.Transaction] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.DmiScaleFactor, info.DmiScaleFactor] = ros.internal.ros.messages.ros.default_type('single',1);
[data.DmiLeverArm, info.DmiLeverArm] = ros.internal.ros.messages.geometry_msgs.point32;
info.DmiLeverArm.MLdataType = 'struct';
[data.Reserved1, info.Reserved1] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Reserved2, info.Reserved2] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Reserved3, info.Reserved3] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Reserved5, info.Reserved5] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Reserved6, info.Reserved6] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Reserved7, info.Reserved7] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'applanix_msgs/AidingSensorParams';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'transaction';
info.MatPath{2} = 'dmi_scale_factor';
info.MatPath{3} = 'dmi_lever_arm';
info.MatPath{4} = 'dmi_lever_arm.x';
info.MatPath{5} = 'dmi_lever_arm.y';
info.MatPath{6} = 'dmi_lever_arm.z';
info.MatPath{7} = 'reserved1';
info.MatPath{8} = 'reserved2';
info.MatPath{9} = 'reserved3';
info.MatPath{10} = 'reserved5';
info.MatPath{11} = 'reserved6';
info.MatPath{12} = 'reserved7';
