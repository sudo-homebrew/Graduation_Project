function [data, info] = trackLinkCmd
%TrackLinkCmd gives an empty data for pr2_mechanism_controllers/TrackLinkCmd

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_mechanism_controllers/TrackLinkCmd';
[data.Enable, info.Enable] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.LinkName, info.LinkName] = ros.internal.ros.messages.ros.char('string',0);
[data.Point, info.Point] = ros.internal.ros.messages.geometry_msgs.point;
info.Point.MLdataType = 'struct';
info.MessageType = 'pr2_mechanism_controllers/TrackLinkCmd';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'enable';
info.MatPath{2} = 'link_name';
info.MatPath{3} = 'point';
info.MatPath{4} = 'point.x';
info.MatPath{5} = 'point.y';
info.MatPath{6} = 'point.z';
