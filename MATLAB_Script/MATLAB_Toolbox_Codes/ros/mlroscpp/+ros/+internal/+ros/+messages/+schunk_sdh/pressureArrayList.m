function [data, info] = pressureArrayList
%PressureArrayList gives an empty data for schunk_sdh/PressureArrayList

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'schunk_sdh/PressureArrayList';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.PressureList, info.PressureList] = ros.internal.ros.messages.schunk_sdh.pressureArray;
info.PressureList.MLdataType = 'struct';
info.PressureList.MaxLen = NaN;
info.PressureList.MinLen = 0;
data.PressureList = data.PressureList([],1);
info.MessageType = 'schunk_sdh/PressureArrayList';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'pressure_list';
info.MatPath{8} = 'pressure_list.sensor_name';
info.MatPath{9} = 'pressure_list.cells_x';
info.MatPath{10} = 'pressure_list.cells_y';
info.MatPath{11} = 'pressure_list.pressure';
