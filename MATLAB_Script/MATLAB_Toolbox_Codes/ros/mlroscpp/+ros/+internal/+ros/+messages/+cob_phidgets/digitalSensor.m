function [data, info] = digitalSensor
%DigitalSensor gives an empty data for cob_phidgets/DigitalSensor

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_phidgets/DigitalSensor';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Uri, info.Uri] = ros.internal.ros.messages.ros.char('string',NaN);
[data.State, info.State] = ros.internal.ros.messages.ros.default_type('int8',NaN);
info.MessageType = 'cob_phidgets/DigitalSensor';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'uri';
info.MatPath{8} = 'state';
