function [data, info] = matrix33
%matrix33 gives an empty data for ardrone_autonomy/matrix33

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ardrone_autonomy/matrix33';
[data.M11, info.M11] = ros.internal.ros.messages.ros.default_type('single',1);
[data.M12, info.M12] = ros.internal.ros.messages.ros.default_type('single',1);
[data.M13, info.M13] = ros.internal.ros.messages.ros.default_type('single',1);
[data.M21, info.M21] = ros.internal.ros.messages.ros.default_type('single',1);
[data.M22, info.M22] = ros.internal.ros.messages.ros.default_type('single',1);
[data.M23, info.M23] = ros.internal.ros.messages.ros.default_type('single',1);
[data.M31, info.M31] = ros.internal.ros.messages.ros.default_type('single',1);
[data.M32, info.M32] = ros.internal.ros.messages.ros.default_type('single',1);
[data.M33, info.M33] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'ardrone_autonomy/matrix33';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'm11';
info.MatPath{2} = 'm12';
info.MatPath{3} = 'm13';
info.MatPath{4} = 'm21';
info.MatPath{5} = 'm22';
info.MatPath{6} = 'm23';
info.MatPath{7} = 'm31';
info.MatPath{8} = 'm32';
info.MatPath{9} = 'm33';
