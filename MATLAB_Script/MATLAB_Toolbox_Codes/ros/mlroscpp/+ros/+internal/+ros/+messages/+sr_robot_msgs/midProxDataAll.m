function [data, info] = midProxDataAll
%MidProxDataAll gives an empty data for sr_robot_msgs/MidProxDataAll

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/MidProxDataAll';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Sensors, info.Sensors] = ros.internal.ros.messages.sr_robot_msgs.midProxData;
info.Sensors.MLdataType = 'struct';
info.Sensors.MaxLen = 5;
info.Sensors.MinLen = 5;
val = [];
for i = 1:5
    val = vertcat(data.Sensors, val); %#ok<AGROW>
end
data.Sensors = val;
info.MessageType = 'sr_robot_msgs/MidProxDataAll';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'sensors';
info.MatPath{8} = 'sensors.middle';
info.MatPath{9} = 'sensors.proximal';