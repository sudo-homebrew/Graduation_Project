function [data, info] = getPolledImageResponse
%GetPolledImage gives an empty data for polled_camera/GetPolledImageResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'polled_camera/GetPolledImageResponse';
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.StatusMessage, info.StatusMessage] = ros.internal.ros.messages.ros.char('string',0);
[data.Stamp, info.Stamp] = ros.internal.ros.messages.ros.time;
info.Stamp.MLdataType = 'struct';
info.MessageType = 'polled_camera/GetPolledImageResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'success';
info.MatPath{2} = 'status_message';
info.MatPath{3} = 'stamp';
info.MatPath{4} = 'stamp.sec';
info.MatPath{5} = 'stamp.nsec';
