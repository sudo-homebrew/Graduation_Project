function [data, info] = routeSegment
%RouteSegment gives an empty data for geographic_msgs/RouteSegment

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geographic_msgs/RouteSegment';
[data.Id, info.Id] = ros.internal.ros.messages.uuid_msgs.uniqueID;
info.Id.MLdataType = 'struct';
[data.Start, info.Start] = ros.internal.ros.messages.uuid_msgs.uniqueID;
info.Start.MLdataType = 'struct';
[data.End, info.End] = ros.internal.ros.messages.uuid_msgs.uniqueID;
info.End.MLdataType = 'struct';
[data.Props, info.Props] = ros.internal.ros.messages.geographic_msgs.keyValue;
info.Props.MLdataType = 'struct';
info.Props.MaxLen = NaN;
info.Props.MinLen = 0;
data.Props = data.Props([],1);
info.MessageType = 'geographic_msgs/RouteSegment';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'id';
info.MatPath{2} = 'id.uuid';
info.MatPath{3} = 'start';
info.MatPath{4} = 'start.uuid';
info.MatPath{5} = 'end';
info.MatPath{6} = 'end.uuid';
info.MatPath{7} = 'props';
info.MatPath{8} = 'props.key';
info.MatPath{9} = 'props.value';
