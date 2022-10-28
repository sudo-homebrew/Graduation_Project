function [data, info] = odometryMatrix
%OdometryMatrix gives an empty data for pr2_mechanism_controllers/OdometryMatrix

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_mechanism_controllers/OdometryMatrix';
[data.M, info.M] = ros.internal.ros.messages.ros.default_type('double',NaN);
info.MessageType = 'pr2_mechanism_controllers/OdometryMatrix';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'm';
