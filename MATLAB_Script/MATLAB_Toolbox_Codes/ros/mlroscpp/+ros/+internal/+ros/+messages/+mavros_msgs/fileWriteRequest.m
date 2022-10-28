function [data, info] = fileWriteRequest
%FileWrite gives an empty data for mavros_msgs/FileWriteRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/FileWriteRequest';
[data.FilePath, info.FilePath] = ros.internal.ros.messages.ros.char('string',0);
[data.Offset, info.Offset] = ros.internal.ros.messages.ros.default_type('uint64',1);
[data.Data, info.Data] = ros.internal.ros.messages.ros.default_type('uint8',NaN);
info.MessageType = 'mavros_msgs/FileWriteRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'file_path';
info.MatPath{2} = 'offset';
info.MatPath{3} = 'data';
