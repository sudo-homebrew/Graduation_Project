function [data, info] = relaxRequest
%Relax gives an empty data for arbotix_msgs/RelaxRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'arbotix_msgs/RelaxRequest';
info.MessageType = 'arbotix_msgs/RelaxRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
