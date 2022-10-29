function [data, info] = changeModeRequest
%ChangeMode gives an empty data for image_view2/ChangeModeRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'image_view2/ChangeModeRequest';
[data.Mode, info.Mode] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'image_view2/ChangeModeRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'mode';
