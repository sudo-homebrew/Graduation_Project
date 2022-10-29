function [data, info] = blob
%Blob gives an empty data for cmvision/Blob

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cmvision/Blob';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Red, info.Red] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Green, info.Green] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Blue, info.Blue] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Area, info.Area] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.X, info.X] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Y, info.Y] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Left, info.Left] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Right, info.Right] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Top, info.Top] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Bottom, info.Bottom] = ros.internal.ros.messages.ros.default_type('uint32',1);
info.MessageType = 'cmvision/Blob';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'name';
info.MatPath{2} = 'red';
info.MatPath{3} = 'green';
info.MatPath{4} = 'blue';
info.MatPath{5} = 'area';
info.MatPath{6} = 'x';
info.MatPath{7} = 'y';
info.MatPath{8} = 'left';
info.MatPath{9} = 'right';
info.MatPath{10} = 'top';
info.MatPath{11} = 'bottom';
