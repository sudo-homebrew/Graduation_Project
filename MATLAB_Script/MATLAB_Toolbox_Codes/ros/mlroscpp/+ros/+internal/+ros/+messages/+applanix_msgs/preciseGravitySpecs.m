function [data, info] = preciseGravitySpecs
%PreciseGravitySpecs gives an empty data for applanix_msgs/PreciseGravitySpecs

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/PreciseGravitySpecs';
[data.Transaction, info.Transaction] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.GravityMagnitude, info.GravityMagnitude] = ros.internal.ros.messages.ros.default_type('double',1);
[data.NorthDeflection, info.NorthDeflection] = ros.internal.ros.messages.ros.default_type('double',1);
[data.EastDeflection, info.EastDeflection] = ros.internal.ros.messages.ros.default_type('double',1);
[data.LatitudeOfValidity, info.LatitudeOfValidity] = ros.internal.ros.messages.ros.default_type('double',1);
[data.LongitudeOfValidity, info.LongitudeOfValidity] = ros.internal.ros.messages.ros.default_type('double',1);
[data.AltitudeOfValidity, info.AltitudeOfValidity] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'applanix_msgs/PreciseGravitySpecs';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'transaction';
info.MatPath{2} = 'gravity_magnitude';
info.MatPath{3} = 'north_deflection';
info.MatPath{4} = 'east_deflection';
info.MatPath{5} = 'latitude_of_validity';
info.MatPath{6} = 'longitude_of_validity';
info.MatPath{7} = 'altitude_of_validity';
