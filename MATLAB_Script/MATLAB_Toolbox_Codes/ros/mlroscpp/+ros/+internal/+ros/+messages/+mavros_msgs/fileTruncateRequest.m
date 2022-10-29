function [data, info] = fileTruncateRequest
%FileTruncate gives an empty data for mavros_msgs/FileTruncateRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/FileTruncateRequest';
[data.FilePath, info.FilePath] = ros.internal.ros.messages.ros.char('string',0);
[data.Length, info.Length] = ros.internal.ros.messages.ros.default_type('uint64',1);
info.MessageType = 'mavros_msgs/FileTruncateRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'file_path';
info.MatPath{2} = 'length';
