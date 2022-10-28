function [data, info] = floats
%Floats gives an empty data for rospy_tutorials/Floats

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rospy_tutorials/Floats';
[data.Data, info.Data] = ros.internal.ros.messages.ros.default_type('single',NaN);
info.MessageType = 'rospy_tutorials/Floats';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'data';
