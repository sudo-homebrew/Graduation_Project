function [data, info] = deleteTFResponse
%DeleteTF gives an empty data for dynamic_tf_publisher/DeleteTFResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'dynamic_tf_publisher/DeleteTFResponse';
info.MessageType = 'dynamic_tf_publisher/DeleteTFResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
