function [data, info] = dissocTFResponse
%DissocTF gives an empty data for dynamic_tf_publisher/DissocTFResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'dynamic_tf_publisher/DissocTFResponse';
info.MessageType = 'dynamic_tf_publisher/DissocTFResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
