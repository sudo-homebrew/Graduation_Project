function [data, info] = fileChecksumRequest
%FileChecksum gives an empty data for mavros_msgs/FileChecksumRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/FileChecksumRequest';
[data.FilePath, info.FilePath] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'mavros_msgs/FileChecksumRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'file_path';
