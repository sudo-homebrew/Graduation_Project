function [data, info] = resetMappingResponse
%ResetMapping gives an empty data for hector_mapping/ResetMappingResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'hector_mapping/ResetMappingResponse';
info.MessageType = 'hector_mapping/ResetMappingResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
