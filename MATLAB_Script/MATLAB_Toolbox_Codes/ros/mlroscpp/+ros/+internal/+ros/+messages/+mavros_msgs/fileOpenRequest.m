function [data, info] = fileOpenRequest
%FileOpen gives an empty data for mavros_msgs/FileOpenRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/FileOpenRequest';
[data.MODEREAD, info.MODEREAD] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.MODEWRITE, info.MODEWRITE] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.MODECREATE, info.MODECREATE] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.FilePath, info.FilePath] = ros.internal.ros.messages.ros.char('string',0);
[data.Mode, info.Mode] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'mavros_msgs/FileOpenRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'MODE_READ';
info.MatPath{2} = 'MODE_WRITE';
info.MatPath{3} = 'MODE_CREATE';
info.MatPath{4} = 'file_path';
info.MatPath{5} = 'mode';
