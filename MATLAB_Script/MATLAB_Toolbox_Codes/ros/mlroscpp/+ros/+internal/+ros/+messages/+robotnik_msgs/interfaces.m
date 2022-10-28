function [data, info] = interfaces
%Interfaces gives an empty data for robotnik_msgs/Interfaces

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/Interfaces';
[data.Total, info.Total] = ros.internal.ros.messages.robotnik_msgs.data;
info.Total.MLdataType = 'struct';
[data.Interfaces_, info.Interfaces_] = ros.internal.ros.messages.robotnik_msgs.data;
info.Interfaces_.MLdataType = 'struct';
info.Interfaces_.MaxLen = NaN;
info.Interfaces_.MinLen = 0;
data.Interfaces_ = data.Interfaces_([],1);
info.MessageType = 'robotnik_msgs/Interfaces';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,16);
info.MatPath{1} = 'total';
info.MatPath{2} = 'total.interface';
info.MatPath{3} = 'total.bytes_sent';
info.MatPath{4} = 'total.bytes_received';
info.MatPath{5} = 'total.units_sent';
info.MatPath{6} = 'total.units_received';
info.MatPath{7} = 'total.packages_sent';
info.MatPath{8} = 'total.packages_received';
info.MatPath{9} = 'interfaces';
info.MatPath{10} = 'interfaces.interface';
info.MatPath{11} = 'interfaces.bytes_sent';
info.MatPath{12} = 'interfaces.bytes_received';
info.MatPath{13} = 'interfaces.units_sent';
info.MatPath{14} = 'interfaces.units_received';
info.MatPath{15} = 'interfaces.packages_sent';
info.MatPath{16} = 'interfaces.packages_received';
