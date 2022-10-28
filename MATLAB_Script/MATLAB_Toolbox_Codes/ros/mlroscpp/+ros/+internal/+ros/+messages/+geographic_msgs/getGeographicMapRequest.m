function [data, info] = getGeographicMapRequest
%GetGeographicMap gives an empty data for geographic_msgs/GetGeographicMapRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geographic_msgs/GetGeographicMapRequest';
[data.Url, info.Url] = ros.internal.ros.messages.ros.char('string',0);
[data.Bounds, info.Bounds] = ros.internal.ros.messages.geographic_msgs.boundingBox;
info.Bounds.MLdataType = 'struct';
info.MessageType = 'geographic_msgs/GetGeographicMapRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'url';
info.MatPath{2} = 'bounds';
info.MatPath{3} = 'bounds.min_pt';
info.MatPath{4} = 'bounds.min_pt.latitude';
info.MatPath{5} = 'bounds.min_pt.longitude';
info.MatPath{6} = 'bounds.min_pt.altitude';
info.MatPath{7} = 'bounds.max_pt';
info.MatPath{8} = 'bounds.max_pt.latitude';
info.MatPath{9} = 'bounds.max_pt.longitude';
info.MatPath{10} = 'bounds.max_pt.altitude';
