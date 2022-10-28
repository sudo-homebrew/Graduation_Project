function [data, info] = setProfileRequest
%SetProfile gives an empty data for pr2_mechanism_controllers/SetProfileRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_mechanism_controllers/SetProfileRequest';
[data.UpperTurnaround, info.UpperTurnaround] = ros.internal.ros.messages.ros.default_type('double',1);
[data.LowerTurnaround, info.LowerTurnaround] = ros.internal.ros.messages.ros.default_type('double',1);
[data.UpperDecelBuffer, info.UpperDecelBuffer] = ros.internal.ros.messages.ros.default_type('double',1);
[data.LowerDecelBuffer, info.LowerDecelBuffer] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Profile, info.Profile] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Period, info.Period] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Amplitude, info.Amplitude] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Offset, info.Offset] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'pr2_mechanism_controllers/SetProfileRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'UpperTurnaround';
info.MatPath{2} = 'LowerTurnaround';
info.MatPath{3} = 'upperDecelBuffer';
info.MatPath{4} = 'lowerDecelBuffer';
info.MatPath{5} = 'profile';
info.MatPath{6} = 'period';
info.MatPath{7} = 'amplitude';
info.MatPath{8} = 'offset';
