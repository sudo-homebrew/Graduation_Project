function [data, info] = setTemplateResponse
%SetTemplate gives an empty data for jsk_perception/SetTemplateResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_perception/SetTemplateResponse';
info.MessageType = 'jsk_perception/SetTemplateResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);