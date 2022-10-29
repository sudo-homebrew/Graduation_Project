function [data, info] = wayPoint
%WayPoint gives an empty data for geographic_msgs/WayPoint

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geographic_msgs/WayPoint';
[data.Id, info.Id] = ros.internal.ros.messages.uuid_msgs.uniqueID;
info.Id.MLdataType = 'struct';
[data.Position, info.Position] = ros.internal.ros.messages.geographic_msgs.geoPoint;
info.Position.MLdataType = 'struct';
[data.Props, info.Props] = ros.internal.ros.messages.geographic_msgs.keyValue;
info.Props.MLdataType = 'struct';
info.Props.MaxLen = NaN;
info.Props.MinLen = 0;
data.Props = data.Props([],1);
info.MessageType = 'geographic_msgs/WayPoint';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'id';
info.MatPath{2} = 'id.uuid';
info.MatPath{3} = 'position';
info.MatPath{4} = 'position.latitude';
info.MatPath{5} = 'position.longitude';
info.MatPath{6} = 'position.altitude';
info.MatPath{7} = 'props';
info.MatPath{8} = 'props.key';
info.MatPath{9} = 'props.value';
