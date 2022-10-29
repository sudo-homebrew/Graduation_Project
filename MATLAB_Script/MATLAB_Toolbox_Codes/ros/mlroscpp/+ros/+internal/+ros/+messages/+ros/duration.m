function [data, info] = duration
%ros/Duration gives an empty data for ros/Duration

% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
    data = struct();
    info = struct();
    [data.Sec, info.Sec] = ros.internal.ros.messages.ros.default_type('int32',1);
    [data.Nsec, info.Nsec] = ros.internal.ros.messages.ros.default_type('int32',1);
    info.MessageType = 'ros/Duration';
    info.MLdataType = 'struct';
    info.constant = 0;
    info.default = 0;
    info.maxstrlen = NaN;
    info.MaxLen = 1;
    info.MinLen = 1;
    info.MatPath = {'sec', 'nsec'};
