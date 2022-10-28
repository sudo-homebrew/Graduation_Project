function [data, info] = data
%Data gives an empty data for robotnik_msgs/Data

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/Data';
[data.Interface, info.Interface] = ros.internal.ros.messages.ros.char('string',0);
[data.BytesSent, info.BytesSent] = ros.internal.ros.messages.ros.default_type('single',1);
[data.BytesReceived, info.BytesReceived] = ros.internal.ros.messages.ros.default_type('single',1);
[data.UnitsSent, info.UnitsSent] = ros.internal.ros.messages.ros.char('string',0);
[data.UnitsReceived, info.UnitsReceived] = ros.internal.ros.messages.ros.char('string',0);
[data.PackagesSent, info.PackagesSent] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.PackagesReceived, info.PackagesReceived] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'robotnik_msgs/Data';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'interface';
info.MatPath{2} = 'bytes_sent';
info.MatPath{3} = 'bytes_received';
info.MatPath{4} = 'units_sent';
info.MatPath{5} = 'units_received';
info.MatPath{6} = 'packages_sent';
info.MatPath{7} = 'packages_received';
