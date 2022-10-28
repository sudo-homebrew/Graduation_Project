function [data, info] = fileEntry
%FileEntry gives an empty data for mavros_msgs/FileEntry

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/FileEntry';
[data.TYPEFILE, info.TYPEFILE] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.TYPEDIRECTORY, info.TYPEDIRECTORY] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Type, info.Type] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Size, info.Size] = ros.internal.ros.messages.ros.default_type('uint64',1);
info.MessageType = 'mavros_msgs/FileEntry';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'TYPE_FILE';
info.MatPath{2} = 'TYPE_DIRECTORY';
info.MatPath{3} = 'name';
info.MatPath{4} = 'type';
info.MatPath{5} = 'size';
