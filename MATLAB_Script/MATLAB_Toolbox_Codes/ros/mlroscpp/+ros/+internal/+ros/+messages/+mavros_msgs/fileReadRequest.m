function [data, info] = fileReadRequest
%FileRead gives an empty data for mavros_msgs/FileReadRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/FileReadRequest';
[data.FilePath, info.FilePath] = ros.internal.ros.messages.ros.char('string',0);
[data.Offset, info.Offset] = ros.internal.ros.messages.ros.default_type('uint64',1);
[data.Size, info.Size] = ros.internal.ros.messages.ros.default_type('uint64',1);
info.MessageType = 'mavros_msgs/FileReadRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'file_path';
info.MatPath{2} = 'offset';
info.MatPath{3} = 'size';
