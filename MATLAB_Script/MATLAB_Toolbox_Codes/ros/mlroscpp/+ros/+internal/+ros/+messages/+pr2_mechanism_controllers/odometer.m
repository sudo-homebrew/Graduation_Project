function [data, info] = odometer
%Odometer gives an empty data for pr2_mechanism_controllers/Odometer

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_mechanism_controllers/Odometer';
[data.Distance, info.Distance] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Angle, info.Angle] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'pr2_mechanism_controllers/Odometer';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'distance';
info.MatPath{2} = 'angle';
