function [data, info] = clearpathRobot
%ClearpathRobot gives an empty data for clearpath_base/ClearpathRobot

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'clearpath_base/ClearpathRobot';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Model, info.Model] = ros.internal.ros.messages.ros.char('string',0);
[data.PlatformRevision, info.PlatformRevision] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Serial, info.Serial] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.HorizonVersion, info.HorizonVersion] = ros.internal.ros.messages.ros.default_type('int32',2);
[data.FirmwareVersion, info.FirmwareVersion] = ros.internal.ros.messages.ros.default_type('int32',2);
[data.FirmwareRevision, info.FirmwareRevision] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.WriteDate, info.WriteDate] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'clearpath_base/ClearpathRobot';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'name';
info.MatPath{2} = 'model';
info.MatPath{3} = 'platform_revision';
info.MatPath{4} = 'serial';
info.MatPath{5} = 'horizon_version';
info.MatPath{6} = 'firmware_version';
info.MatPath{7} = 'firmware_revision';
info.MatPath{8} = 'write_date';
