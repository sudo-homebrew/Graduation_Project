function [data, info] = getFootprintRequest
%GetFootprint gives an empty data for cob_footprint_observer/GetFootprintRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_footprint_observer/GetFootprintRequest';
info.MessageType = 'cob_footprint_observer/GetFootprintRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
