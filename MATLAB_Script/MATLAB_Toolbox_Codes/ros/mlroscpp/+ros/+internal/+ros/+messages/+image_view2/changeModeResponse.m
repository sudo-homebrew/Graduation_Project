function [data, info] = changeModeResponse
%ChangeMode gives an empty data for image_view2/ChangeModeResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'image_view2/ChangeModeResponse';
info.MessageType = 'image_view2/ChangeModeResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
