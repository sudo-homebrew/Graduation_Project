function [data, info] = biotacAll
%BiotacAll gives an empty data for sr_robot_msgs/BiotacAll

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/BiotacAll';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Tactiles, info.Tactiles] = ros.internal.ros.messages.sr_robot_msgs.biotac;
info.Tactiles.MLdataType = 'struct';
info.Tactiles.MaxLen = 5;
info.Tactiles.MinLen = 5;
val = [];
for i = 1:5
    val = vertcat(data.Tactiles, val); %#ok<AGROW>
end
data.Tactiles = val;
info.MessageType = 'sr_robot_msgs/BiotacAll';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,13);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'tactiles';
info.MatPath{8} = 'tactiles.pac0';
info.MatPath{9} = 'tactiles.pac1';
info.MatPath{10} = 'tactiles.pdc';
info.MatPath{11} = 'tactiles.tac';
info.MatPath{12} = 'tactiles.tdc';
info.MatPath{13} = 'tactiles.electrodes';
