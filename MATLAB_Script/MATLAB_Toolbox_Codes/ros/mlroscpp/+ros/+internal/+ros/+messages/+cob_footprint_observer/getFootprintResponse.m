function [data, info] = getFootprintResponse
%GetFootprint gives an empty data for cob_footprint_observer/GetFootprintResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_footprint_observer/GetFootprintResponse';
[data.Footprint, info.Footprint] = ros.internal.ros.messages.geometry_msgs.polygonStamped;
info.Footprint.MLdataType = 'struct';
[data.Success, info.Success] = ros.internal.ros.messages.std_msgs.bool;
info.Success.MLdataType = 'struct';
info.MessageType = 'cob_footprint_observer/GetFootprintResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'footprint';
info.MatPath{2} = 'footprint.header';
info.MatPath{3} = 'footprint.header.seq';
info.MatPath{4} = 'footprint.header.stamp';
info.MatPath{5} = 'footprint.header.stamp.sec';
info.MatPath{6} = 'footprint.header.stamp.nsec';
info.MatPath{7} = 'footprint.header.frame_id';
info.MatPath{8} = 'footprint.polygon';
info.MatPath{9} = 'footprint.polygon.points';
info.MatPath{10} = 'footprint.polygon.points.x';
info.MatPath{11} = 'footprint.polygon.points.y';
info.MatPath{12} = 'footprint.polygon.points.z';
info.MatPath{13} = 'success';
info.MatPath{14} = 'success.data';
