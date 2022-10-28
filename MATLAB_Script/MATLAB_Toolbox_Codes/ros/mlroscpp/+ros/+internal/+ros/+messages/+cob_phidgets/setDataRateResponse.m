function [data, info] = setDataRateResponse
%SetDataRate gives an empty data for cob_phidgets/SetDataRateResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_phidgets/SetDataRateResponse';
info.MessageType = 'cob_phidgets/SetDataRateResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
