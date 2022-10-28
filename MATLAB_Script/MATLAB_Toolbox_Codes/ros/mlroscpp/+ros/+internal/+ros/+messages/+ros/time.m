function [data, info] = time
%ros_msg_Time gives an empty data for ros/Time

% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
    data = struct();
    info = struct();
    [data.Sec, info.Sec] = ros.internal.ros.messages.ros.default_type('uint32',1);
    [data.Nsec, info.Nsec] = ros.internal.ros.messages.ros.default_type('uint32',1);
    info.MessageType = 'ros/Time';
    info.MLdataType = 'struct';
    info.constant = 0;
    info.default = 0;
    info.maxstrlen = NaN;
    info.MaxLen = 1;
    info.MinLen = 1;
    info.MatPath = {'sec', 'nsec'};
