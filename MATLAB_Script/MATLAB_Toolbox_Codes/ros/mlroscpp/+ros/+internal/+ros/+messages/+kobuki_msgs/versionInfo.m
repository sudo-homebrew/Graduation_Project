function [data, info] = versionInfo
%VersionInfo gives an empty data for kobuki_msgs/VersionInfo

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'kobuki_msgs/VersionInfo';
[data.Hardware, info.Hardware] = ros.internal.ros.messages.ros.char('string',0);
[data.Firmware, info.Firmware] = ros.internal.ros.messages.ros.char('string',0);
[data.Software, info.Software] = ros.internal.ros.messages.ros.char('string',0);
[data.Udid, info.Udid] = ros.internal.ros.messages.ros.default_type('uint32',NaN);
[data.Features, info.Features] = ros.internal.ros.messages.ros.default_type('uint64',1);
[data.SMOOTHMOVESTART, info.SMOOTHMOVESTART] = ros.internal.ros.messages.ros.default_type('uint64',1, 1);
[data.GYROSCOPE3DDATA, info.GYROSCOPE3DDATA] = ros.internal.ros.messages.ros.default_type('uint64',1, 2);
info.MessageType = 'kobuki_msgs/VersionInfo';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'hardware';
info.MatPath{2} = 'firmware';
info.MatPath{3} = 'software';
info.MatPath{4} = 'udid';
info.MatPath{5} = 'features';
info.MatPath{6} = 'SMOOTH_MOVE_START';
info.MatPath{7} = 'GYROSCOPE_3D_DATA';
